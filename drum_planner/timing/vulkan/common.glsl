#extension GL_ARB_gpu_shader_fp64 : require
layout(local_size_x=64) in;
layout(std430,binding=0) buffer Data { double a[]; };
layout(push_constant) uniform Params { uint n; uint m; uint unused; } p;
const uint K=24, L=8;
uint X(){return 64;}
uint G(){return X()+p.n;}
uint D(){return G()+p.n;}
uint OX(){return D()+p.n;}
uint OG(){return OX()+p.n;}
uint W(){return OG()+p.n;}
uint T(){return W()+p.n;}
uint S(){return T()+p.n;}
uint Y(){return S()+L*p.n;}
uint SY(){return Y()+L*p.n;}
uint YY(){return SY()+L;}
uint AL(){return YY()+L;}
uint F(){return AL()+L;}
uint V(){return F()+K;}
uint B(){return V()+K;}
uint Q(){return B()+3*p.n*p.m;}
uint LO(){return Q()+21*p.m;}
uint HI(){return LO()+p.n;}
uint ROW(){return HI()+p.n;}
uint LM(){return ROW()+13*p.m;}
uint AUX(){return LM()+21*p.m;}
double step(uint c){return c==0?0.0:1.0/double(1u<<(c-1));}
double candidate(uint c,uint j){return clamp(a[X()+j]+step(c)*a[D()+j],0.005,5.0);}
shared double work[64];
double sum64(double v){uint l=gl_LocalInvocationID.x;work[l]=v;barrier();for(uint d=32;d>0;d>>=1){if(l<d)work[l]+=work[l+d];barrier();}double outv=work[0];barrier();return outv;}
double max64(double v){uint l=gl_LocalInvocationID.x;work[l]=v;barrier();for(uint d=32;d>0;d>>=1){if(l<d)work[l]=max(work[l],work[l+d]);barrier();}double outv=work[0];barrier();return outv;}
void syncbuf(){memoryBarrierBuffer();barrier();}
