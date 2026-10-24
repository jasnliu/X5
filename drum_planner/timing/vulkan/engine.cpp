// Experiment-only FP64 GPU-native augmented-Lagrangian / L-BFGS engine.
// All optimizer updates run in shaders. No motors, cameras or software-GPU fallback.
#include <vulkan/vulkan.h>
#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

static thread_local std::string error;
static thread_local std::string identity;
static void check(VkResult r, const char* op) {
    if(r!=VK_SUCCESS) throw std::runtime_error(std::string(op)+": Vulkan error "+std::to_string(r));
}
struct Engine {
    VkInstance instance{}; VkPhysicalDevice physical{}; VkDevice device{};
    VkQueue queue{}; uint32_t family{}, n{}, m{};
    VkPhysicalDeviceProperties properties{};
    VkBuffer buffer{}; VkDeviceMemory memory{}; double* mapped{};
    VkDescriptorSetLayout descriptors{}; VkDescriptorPool descriptorPool{};
    VkDescriptorSet descriptorSet{}; VkPipelineLayout layout{};
    std::array<VkPipeline,6> pipelines{};
    VkCommandPool commands{}; std::array<VkCommandBuffer,2> command{};
    VkFence fence{};
    size_t total{};

    ~Engine() {
        if(device) {
            vkDeviceWaitIdle(device);
            if(fence) vkDestroyFence(device,fence,nullptr);
            if(commands) vkDestroyCommandPool(device,commands,nullptr);
            for(auto p:pipelines) if(p) vkDestroyPipeline(device,p,nullptr);
            if(layout) vkDestroyPipelineLayout(device,layout,nullptr);
            if(descriptorPool) vkDestroyDescriptorPool(device,descriptorPool,nullptr);
            if(descriptors) vkDestroyDescriptorSetLayout(device,descriptors,nullptr);
            if(mapped) vkUnmapMemory(device,memory);
            if(buffer) vkDestroyBuffer(device,buffer,nullptr);
            if(memory) vkFreeMemory(device,memory,nullptr);
            vkDestroyDevice(device,nullptr);
        }
        if(instance) vkDestroyInstance(instance,nullptr);
    }
    void initialize(uint32_t controls,uint32_t rows,const double* data,size_t countDoubles,const char* shaderDir) {
        n=controls; m=rows;
        if(n<4 || !m || n>192 || m>10000) throw std::runtime_error("Invalid bounded timing size");
        VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO}; app.pApplicationName="X5 experiment GPU timing"; app.apiVersion=VK_API_VERSION_1_2;
        VkInstanceCreateInfo ic{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO}; ic.pApplicationInfo=&app;
        check(vkCreateInstance(&ic,nullptr,&instance),"create instance");
        uint32_t count=0; check(vkEnumeratePhysicalDevices(instance,&count,nullptr),"enumerate devices");
        std::vector<VkPhysicalDevice> devices(count); check(vkEnumeratePhysicalDevices(instance,&count,devices.data()),"enumerate devices");
        for(auto p:devices) {
            VkPhysicalDeviceProperties prop; vkGetPhysicalDeviceProperties(p,&prop);
            if(prop.vendorID==0x1002 && prop.deviceType==VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU) {physical=p;properties=prop;break;}
        }
        if(!physical) throw std::runtime_error("AMD integrated GPU required; refusing llvmpipe");
        VkPhysicalDeviceFeatures features; vkGetPhysicalDeviceFeatures(physical,&features);
        if(!features.shaderFloat64) throw std::runtime_error("GPU FP64 unsupported");
        uint32_t qc=0; vkGetPhysicalDeviceQueueFamilyProperties(physical,&qc,nullptr);
        std::vector<VkQueueFamilyProperties> qp(qc); vkGetPhysicalDeviceQueueFamilyProperties(physical,&qc,qp.data());
        bool found=false;
        for(uint32_t i=0;i<qc;i++) if(qp[i].queueFlags&VK_QUEUE_COMPUTE_BIT) {family=i;found=true;break;}
        if(!found) throw std::runtime_error("No compute queue");
        float priority=0.5f;
        VkDeviceQueueCreateInfo qi{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO}; qi.queueFamilyIndex=family; qi.queueCount=1; qi.pQueuePriorities=&priority;
        VkPhysicalDeviceFeatures enabled{}; enabled.shaderFloat64=VK_TRUE;
        VkDeviceCreateInfo di{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO}; di.queueCreateInfoCount=1; di.pQueueCreateInfos=&qi; di.pEnabledFeatures=&enabled;
        check(vkCreateDevice(physical,&di,nullptr,&device),"create device"); vkGetDeviceQueue(device,family,0,&queue);
        total=countDoubles;
        VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO}; bi.size=total*sizeof(double); bi.usage=VK_BUFFER_USAGE_STORAGE_BUFFER_BIT; bi.sharingMode=VK_SHARING_MODE_EXCLUSIVE;
        check(vkCreateBuffer(device,&bi,nullptr,&buffer),"create buffer");
        VkMemoryRequirements mr; vkGetBufferMemoryRequirements(device,buffer,&mr);
        VkPhysicalDeviceMemoryProperties mp; vkGetPhysicalDeviceMemoryProperties(physical,&mp);
        uint32_t type=UINT32_MAX;
        VkMemoryPropertyFlags wanted=VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT|VK_MEMORY_PROPERTY_HOST_CACHED_BIT;
        for(uint32_t i=0;i<mp.memoryTypeCount;i++) if((mr.memoryTypeBits&(1u<<i)) && (mp.memoryTypes[i].propertyFlags&wanted)==wanted) {type=i;break;}
        if(type==UINT32_MAX) throw std::runtime_error("Requested host-coherent memory unavailable");
        VkMemoryAllocateInfo ma{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO}; ma.allocationSize=mr.size; ma.memoryTypeIndex=type;
        check(vkAllocateMemory(device,&ma,nullptr,&memory),"allocate memory"); check(vkBindBufferMemory(device,buffer,memory,0),"bind memory");
        check(vkMapMemory(device,memory,0,VK_WHOLE_SIZE,0,reinterpret_cast<void**>(&mapped)),"map memory");
        std::memcpy(mapped,data,total*8);
        VkDescriptorSetLayoutBinding binding{}; binding.binding=0; binding.descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; binding.descriptorCount=1; binding.stageFlags=VK_SHADER_STAGE_COMPUTE_BIT;
        VkDescriptorSetLayoutCreateInfo dl{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO}; dl.bindingCount=1; dl.pBindings=&binding;
        check(vkCreateDescriptorSetLayout(device,&dl,nullptr,&descriptors),"descriptor layout");
        VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1};
        VkDescriptorPoolCreateInfo dp{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO}; dp.maxSets=1; dp.poolSizeCount=1; dp.pPoolSizes=&ps;
        check(vkCreateDescriptorPool(device,&dp,nullptr,&descriptorPool),"descriptor pool");
        VkDescriptorSetAllocateInfo da{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO}; da.descriptorPool=descriptorPool; da.descriptorSetCount=1; da.pSetLayouts=&descriptors;
        check(vkAllocateDescriptorSets(device,&da,&descriptorSet),"allocate descriptor");
        VkDescriptorBufferInfo db{buffer,0,VK_WHOLE_SIZE}; VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET}; write.dstSet=descriptorSet; write.dstBinding=0; write.descriptorCount=1; write.descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; write.pBufferInfo=&db;
        vkUpdateDescriptorSets(device,1,&write,0,nullptr);
        VkPushConstantRange push{VK_SHADER_STAGE_COMPUTE_BIT,0,12}; VkPipelineLayoutCreateInfo pl{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO}; pl.setLayoutCount=1; pl.pSetLayouts=&descriptors; pl.pushConstantRangeCount=1; pl.pPushConstantRanges=&push;
        check(vkCreatePipelineLayout(device,&pl,nullptr,&layout),"pipeline layout");
        const char* names[]={"eval","cost","choose","gradient","update","dual"};
        for(int i=0;i<6;i++) {
            std::string path=std::string(shaderDir)+"/"+names[i]+".spv";
            std::ifstream f(path,std::ios::binary|std::ios::ate);
            if(!f) throw std::runtime_error("Cannot open shader "+path);
            size_t bytes=f.tellg(); if(bytes%4) throw std::runtime_error("Invalid SPIR-V size");
            std::vector<uint32_t> code(bytes/4); f.seekg(0); f.read(reinterpret_cast<char*>(code.data()),bytes);
            VkShaderModuleCreateInfo sm{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO}; sm.codeSize=bytes; sm.pCode=code.data(); VkShaderModule module;
            check(vkCreateShaderModule(device,&sm,nullptr,&module),"shader module");
            VkComputePipelineCreateInfo cp{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO}; cp.stage.sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; cp.stage.stage=VK_SHADER_STAGE_COMPUTE_BIT; cp.stage.module=module; cp.stage.pName="main"; cp.layout=layout;
            auto result=vkCreateComputePipelines(device,VK_NULL_HANDLE,1,&cp,nullptr,&pipelines[i]); vkDestroyShaderModule(device,module,nullptr); check(result,"compute pipeline");
        }
        VkCommandPoolCreateInfo pool{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO}; pool.queueFamilyIndex=family;
        check(vkCreateCommandPool(device,&pool,nullptr,&commands),"command pool");
        VkCommandBufferAllocateInfo ca{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO}; ca.commandPool=commands; ca.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY; ca.commandBufferCount=2;
        check(vkAllocateCommandBuffers(device,&ca,command.data()),"allocate commands");
        for(int i=0;i<2;i++) {
            VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO}; check(vkBeginCommandBuffer(command[i],&begin),"begin command");
            std::array<uint32_t,3> params{n,m,0};
            vkCmdBindDescriptorSets(command[i],VK_PIPELINE_BIND_POINT_COMPUTE,layout,0,1,&descriptorSet,0,nullptr);
            vkCmdPushConstants(command[i],layout,VK_SHADER_STAGE_COMPUTE_BIT,0,12,params.data());
            auto dispatch=[&](int pipeline,uint32_t x,uint32_t y){
                vkCmdBindPipeline(command[i],VK_PIPELINE_BIND_POINT_COMPUTE,pipelines[pipeline]);vkCmdDispatch(command[i],x,y,1);
                VkMemoryBarrier b{VK_STRUCTURE_TYPE_MEMORY_BARRIER};b.srcAccessMask=VK_ACCESS_SHADER_WRITE_BIT;b.dstAccessMask=VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT;
                vkCmdPipelineBarrier(command[i],VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,0,1,&b,0,nullptr,0,nullptr);
            };
            if(i==0) for(int k=0;k<32;k++) {dispatch(0,(m+63)/64,24);dispatch(1,24,1);dispatch(2,1,1);dispatch(3,n,1);dispatch(4,1,1);}
            else dispatch(5,(m+63)/64,1);
            VkMemoryBarrier b{VK_STRUCTURE_TYPE_MEMORY_BARRIER};b.srcAccessMask=VK_ACCESS_SHADER_WRITE_BIT;b.dstAccessMask=VK_ACCESS_HOST_READ_BIT;
            vkCmdPipelineBarrier(command[i],VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_HOST_BIT,0,1,&b,0,nullptr,0,nullptr);
            check(vkEndCommandBuffer(command[i]),"end command");
        }
        VkFenceCreateInfo fc{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO}; check(vkCreateFence(device,&fc,nullptr,&fence),"fence");
    }
    void run(int which) {
        check(vkResetFences(device,1,&fence),"reset fence");
        VkSubmitInfo sub{VK_STRUCTURE_TYPE_SUBMIT_INFO};sub.commandBufferCount=1;sub.pCommandBuffers=&command[which];
        check(vkQueueSubmit(queue,1,&sub,fence),"submit");
        check(vkWaitForFences(device,1,&fence,VK_TRUE,5000000000ull),"GPU wait (5 second bound)");
    }

};
extern "C" {
void* vg_create(uint32_t n,uint32_t m,const double* data,size_t count,const char* dir) {
    Engine* e=new Engine;try{e->initialize(n,m,data,count,dir);return e;}catch(const std::exception& ex){error=ex.what();delete e;return nullptr;}
}
void vg_destroy(void* p){delete static_cast<Engine*>(p);}
const char* vg_error(){return error.c_str();}
const char* vg_device(void* p){return static_cast<Engine*>(p)->properties.deviceName;}
const char* vg_identity(void* p){auto& v=static_cast<Engine*>(p)->properties;identity=std::string(v.deviceName)+"/"+std::to_string(v.vendorID)+"/"+std::to_string(v.deviceID)+"/"+std::to_string(v.driverVersion)+"/"+std::to_string(v.apiVersion);return identity.c_str();}
int vg_step(void* p,double* snapshot){try{auto* e=static_cast<Engine*>(p);e->run(0);std::memcpy(snapshot,e->mapped,(64+3*e->n)*8);return 0;}catch(const std::exception& ex){error=ex.what();return -1;}}
int vg_dual(void* p,double nextRho){try{auto* e=static_cast<Engine*>(p);e->run(1);e->mapped[0]=nextRho;return 0;}catch(const std::exception& ex){error=ex.what();return -1;}}
}
