"""Explicit experiment/sparse opt-ins; unchanged default arm-aware CPU timing."""
def retime(winner,report,req,context,folder,log):
    if context.get('timing_backend')=='vulkan':
        if not context.get('experiment_gpu_timing'):
            raise ValueError('Vulkan timing is experiment-only')
        from .gpu_integration import retime as selected
        return selected(winner,report,req,context,folder,log)
    if context.get('timing_backend')=='sparse':
        from .sparse_integration import retime as selected
        return selected(winner,report,req,context,folder,log)
    if req.get('moving_side','right')=='left':
        from .smooth_v2 import retime as selected
    else:
        from .integration import retime as selected
    return selected(winner,report,req,context,folder,log)
