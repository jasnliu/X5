"""Preserve the exact v1 right-arm code path; opt left corner paths into v2."""
def retime(winner,report,req,context,folder,log):
    if req.get('moving_side','right')=='left':
        from .smooth_v2 import retime as selected
    else:
        from .integration import retime as selected
    return selected(winner,report,req,context,folder,log)
