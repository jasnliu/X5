"""Offline Tk integration check for the isolated right-J7 test program."""
import rclpy

from right_joint7_test.app import App


rclpy.init(args=[])
app = None
try:
    app = App(False)
    app.root.withdraw()
    assert app.side == "right"
    assert app.other_side == "left"
    assert app.control_gripper
    assert app.bus is None
    assert app.test_plan is not None
    assert "Joint 7 ten-degree test" in app.root.title()
    assert "CENTER RIGHT ARM" in app.start_button.cget("text")
    assert "FAST J7" in app.continue_button.cget("text")
    assert str(app.continue_button.cget("state")) == "disabled"
    app.start()
    assert "Offline preview only" in app.status.get()
    assert app.bus is None
finally:
    if app is not None:
        app.root.destroy()
        app.node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

print("PASS: right-J7 test UI/preflight loaded offline; no CAN or arm motion")
