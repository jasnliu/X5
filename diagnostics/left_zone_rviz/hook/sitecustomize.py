"""Bounded simulation-only RViz check; no RUN or center commands."""
import os
if os.environ.get('BEAT_ZONE_VERIFY'):
    import inspect,json,socket,sys,time,tkinter
    from pathlib import Path
    def guard(event,args):
        if event=='socket.__new__' and args[1]==socket.AF_CAN:
            raise AssertionError('No physical CAN')
        if event=='open' and isinstance(args[0],str) and args[0].startswith(('/dev/tty','/dev/serial','/dev/video','/dev/snd')):
            raise AssertionError('No physical devices')
    sys.addaudithook(guard)
    original=tkinter.Tk.mainloop
    def verify(root,*args,**kwargs):
        f=inspect.currentframe();app=None
        while f:
            a=f.f_locals.get('self')
            if getattr(a,'root',None) is root and hasattr(a,'center_relax'):
                app=a;break
            f=f.f_back
        if app is None:return original(root,*args,**kwargs)
        assert app.test_mode and type(app.bus).__name__=='SimulatedMotors'
        root.withdraw()
        from visualization_msgs.msg import MarkerArray
        received=[False]
        path=Path(os.environ['BEAT_ZONE_VERIFY'])
        def callback(message):
            left=[m for m in message.markers if m.ns=='left_beat_zone']
            right=[m for m in message.markers if m.ns=='right_goal_motion']
            if received[0] or len(left)!=3 or not right:return
            assert not app.bus.active
            assert any(m.type==11 and len(m.points)>0 for m in left)
            result=dict(simulation_only=True, motors_active=app.bus.active,
                left=[dict(namespace=m.ns,id=m.id,type=m.type,frame=m.header.frame_id,
                           points=len(m.points),text=m.text,alpha=m.color.a) for m in left],
                right_marker_count=len(right),topic='/safe_zone/markers')
            path.write_text(json.dumps(result,indent=2)+'\n')
            print('VERIFIED LIVE ROS: right and left zone markers received; no motors active',flush=True)
            received[0]=True
            root.after(30000,root.quit)
        subscription=app.node.create_subscription(MarkerArray,'/safe_zone/markers',callback,10)
        root.after(90000,root.quit)
        result=original(root,*args,**kwargs)
        assert received[0], 'Left marker publication not received'
        return result
    tkinter.Tk.mainloop=verify
