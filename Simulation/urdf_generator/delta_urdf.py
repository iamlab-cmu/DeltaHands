import numpy as np
from odio_urdf import *
import delta_materials

umass = 0.01 # 10 g
uinertia = [1e-3, 0.0, 0.0, 1e-3, 0.0, 1e-3]
# origin: x, y, z, r, p, y
def base_link(origin, geometry, material):
    link = Link(
        Inertial(Mass(value=umass), Inertia(uinertia)),
        Visual(Origin(origin),Geometry(geometry),Material(material)),
        Collision(Origin(origin), Geometry(geometry),Material(material)),
        name = "dbase_link",
    )
    return link

def base_joint(origin):
    joint = Joint(
        Parent(link = "world"),
        Child(link = "dbase_link"),
        Origin(origin),
        name = "base_joint",
        type = "fixed",
    )
    return joint

def motor_link(N, origin, geometry, material):
    N = str(N)
    link = Link(
        Inertial(Mass(value=umass), Inertia(uinertia)),
        Visual(Origin(origin),Geometry(geometry),Material(material)),
        Collision(Origin(origin), Geometry(geometry),Material(material)),
        name = "motor_link_"+N
    )
    return link

def motor_joint(N, type, origin, limit=None, axis=None):
    N = str(N)
    joint = Joint(
        Axis(axis),
        Limit(lower=limit[0], upper=limit[1], effort=100, velocity=1),
        Parent(link = "dbase_link"),
        Child(link = "motor_link_"+N),
        Origin(origin),
        name = "motor_joint_"+N,
        type = type
    )
    return joint

def leg_link(leg_name, origin, geometry, material):
    link = Link(
        Inertial(Mass(value=umass), Inertia(uinertia)),
        Visual(Origin(origin),Geometry(geometry),Material(material)),
        Collision(Origin(origin), Geometry(geometry),Material(material)),
        name = leg_name
    )
    return link

def leg_joint(name, parent, child, origin, limit=None, axis=None):
    joint = Joint(
        Axis(axis),
        Limit(lower=limit[0], upper=limit[1], effort=100, velocity=1),
        Parent(link = parent),
        Child(link = child),
        Origin(origin),
        name = name,
        type = "revolute"
    )
    return joint

def knee_link(N, origin, geometry, material):
    N = str(N)
    link = Link(
        Inertial(Mass(value=umass), Inertia(uinertia)),
        Visual(Origin(origin),Geometry(geometry),Material(material)),
        Collision(Origin(origin), Geometry(geometry),Material(material)),
        name = "parallel_"+N+"_knee"
    )
    return link

def knee_joint(N, parent, origin, limit=None, axis=None):
    N = str(N)
    joint = Joint(
        Axis(axis),
        Limit(lower=limit[0], upper=limit[1], effort=100, velocity=1),
        Parent(link = parent),
        Child(link = "parallel_"+N+"_knee"),
        Origin(origin),
        name = "parallel_"+N+"_kneejoint",
        type = "revolute"
    )
    return joint

def bar_link(bar_name, origin, geometry, material):
    link = Link(
        Inertial(Mass(value=umass), Inertia(uinertia)),
        Visual(Origin(origin),Geometry(geometry),Material(material)),
        Collision(Origin(origin), Geometry(geometry),Material(material)),
        name = bar_name
    )
    return link

def bar_joint(name, parent, child, origin):
    joint = Joint(
        Parent(link = parent),
        Child(link = child),
        Origin(origin),
        name = name,
        type = "fixed"
    )
    return joint

def parallel_leg(N, origin, parent):
    # include a knee joint, a knee bar, two leg joints, two parallel legs
    N = str(N)
    kknee_joint = knee_joint(N, parent, origin, limit=[-np.pi,np.pi], axis=[1,0,0])
    kknee_link = knee_link(N, [0,0,0,0,0.5*np.pi,0], Cylinder(radius=0.001,length=0.015), "Red")
    leg_joint1 = leg_joint("parallel_"+N+"_legjoint1", "parallel_"+N+"_knee", "parallel_"+N+"_leg1", [-0.005, 0, 0, 0, 0, 0], limit=[-0.5*np.pi,0.5*np.pi], axis=[0, 1, 0])
    leg_joint2 = leg_joint("parallel_"+N+"_legjoint2", "parallel_"+N+"_knee", "parallel_"+N+"_leg2", [0.005, 0, 0, 0, 0, 0], limit=[-0.5*np.pi,0.5*np.pi], axis=[0, 1, 0])
    leg1 = leg_link("parallel_"+N+"_leg1", [0,0,0.02,0,0,0], Box(size=[0.002, 0.002, 0.04]), "white")
    leg2 = leg_link("parallel_"+N+"_leg2", [0,0,0.02,0,0,0], Box(size=[0.002, 0.002, 0.04]), "white")

    return(Group(kknee_joint, kknee_link, leg_joint1, leg_joint2, leg1, leg2))

def double_parallel_leg(N, origin, parent):
    # include a knee joint, a knee bar, 2 x (two leg joints, two parallel legs) in parallel
    N = str(N)
    kknee_joint = knee_joint(N, parent, origin, limit=[-np.pi,np.pi], axis=[1,0,0])
    kknee_link = knee_link(N, [0,0,0,0,0.5*np.pi,0], Cylinder(radius=0.001,length=0.03), "Red")
    leg_joint1 = leg_joint("parallel_"+N+"_legjoint1", "parallel_"+N+"_knee", "parallel_"+N+"1_leg1", [-0.012, 0, 0, 0, 0, 0], limit=[-0.5*np.pi,0.5*np.pi], axis=[0, 1, 0])
    leg_joint2 = leg_joint("parallel_"+N+"_legjoint2", "parallel_"+N+"_knee", "parallel_"+N+"1_leg2", [-0.002, 0, 0, 0, 0, 0], limit=[-0.5*np.pi,0.5*np.pi], axis=[0, 1, 0])
    leg1 = leg_link("parallel_"+N+"1_leg1", [0,0,0.02,0,0,0], Box(size=[0.002, 0.002, 0.04]), "white")
    leg2 = leg_link("parallel_"+N+"1_leg2", [0,0,0.02,0,0,0], Box(size=[0.002, 0.002, 0.04]), "white")

    leg_joint3 = leg_joint("parallel_"+N+"_legjoint3", "parallel_"+N+"_knee", "parallel_"+N+"2_leg1", [0.002, 0, 0, 0, 0, 0], limit=[-0.5*np.pi,0.5*np.pi], axis=[0, 1, 0])
    leg_joint4 = leg_joint("parallel_"+N+"_legjoint4", "parallel_"+N+"_knee", "parallel_"+N+"2_leg2", [0.012, 0, 0, 0, 0, 0], limit=[-0.5*np.pi,0.5*np.pi], axis=[0, 1, 0])
    leg3 = leg_link("parallel_"+N+"2_leg1", [0,0,0.02,0,0,0], Box(size=[0.002, 0.002, 0.04]), "white")
    leg4 = leg_link("parallel_"+N+"2_leg2", [0,0,0.02,0,0,0], Box(size=[0.002, 0.002, 0.04]), "white")

    return(Group(kknee_joint, kknee_link, leg_joint1, leg_joint2, leg1, leg2, leg_joint3, leg_joint4, leg3, leg4))

def linearDelta():
    delta = Group(
        base_joint([0,0,0,0,0,0]),
        base_link([0,0,0,0,0,0], Cylinder(radius=0.03,length=0.01), "Grey"),
        # linear actuator
        motor_joint(1, "prismatic", [0,0,0,0,0,0], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(1, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BrightBlue"),
        # linear actuator
        motor_joint(2, "prismatic", [0.01*np.sqrt(3), -0.01-0.01, 0.0, 0, 0, -np.radians(120)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(2, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BrightBlue"),
        # linear actuator
        motor_joint(3, "prismatic", [-0.01*np.sqrt(3), -0.01-0.01, 0.0, 0, 0, np.radians(120)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(3, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BrightBlue"),

        # parallel leg
        parallel_leg(1, [0,0,0.02,0,0,0], 'motor_link_1'),
        parallel_leg(2, [0,0,0.02,0,0,0], 'motor_link_2'),
        parallel_leg(3, [0,0,0.02,0,0,0], 'motor_link_3'),
    )
    return delta


def twinDelta():
    delta = Group(
        base_joint([0,0,0,0,0,0]),
        base_link([0,0,0,0,0,0], Cylinder(radius=0.03,length=0.01), "Grey"),
        # linear actuator
        motor_joint(1, "prismatic", [0,0,0,0,0,0], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(1, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BrightBlue"),
        # # rotary actuator 2
        # motor_joint(2, "revolute", [0.005*np.sqrt(3), -0.005-0.03, 0.02, 0, 0, -np.radians(120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(2, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # # rotary actuator 3
        # motor_joint(3, "revolute", [-0.005*np.sqrt(3), -0.005-0.03, 0.02, 0, 0, np.radians(120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(3, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # # rotary actuator 4
        # motor_joint(4, "revolute", [0.005*np.sqrt(3), 0.005+0.03, 0.02, 0, 0, -np.radians(60)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(4, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # # rotary actuator 5
        # motor_joint(5, "revolute", [-0.005*np.sqrt(3), 0.005+0.03, 0.02, 0, 0, np.radians(60)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(5, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # linear actuator 2
        motor_joint(2, "prismatic", [0.01*np.sqrt(3), -0.01-0.03, 0.0, 0, 0, -np.radians(120)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(2, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # linear actuator 3
        motor_joint(3, "prismatic", [-0.01*np.sqrt(3), -0.01-0.03, 0.0, 0, 0, np.radians(120)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(3, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # linear actuator 4
        motor_joint(4, "prismatic", [0.01*np.sqrt(3), 0.01+0.03, 0.0, 0, 0, -np.radians(60)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(4, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # linear actuator 5
        motor_joint(5, "prismatic", [-0.01*np.sqrt(3), 0.01+0.03, 0.0, 0, 0, np.radians(60)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(5, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # add bars on top of linear actuator
        bar_joint("bar_joint_1", "motor_link_1", "bar_link_1", [0,0,0.02,np.radians(90),0,0]),
        bar_link("bar_link_1", [0,0,0.0,0,0,0], Cylinder(radius=0.001,length=0.04), "Grey"),
        # parallel leg on linear actuator
        parallel_leg(1, [0,0,0.02,-np.radians(90),0,0], 'bar_link_1'),
        parallel_leg(6, [0,0,-0.02,-np.radians(90),0,0], 'bar_link_1'),
        # parallel leg on rotary actuator
        parallel_leg(2, [0,0,0.02,0,0,0], 'motor_link_2'),
        parallel_leg(3, [0,0,0.02,0,0,0], 'motor_link_3'),
        parallel_leg(4, [0,0,0.02,0,0,0], 'motor_link_4'),
        parallel_leg(5, [0,0,0.02,0,0,0], 'motor_link_5'),
    )
    return delta

def tripleDelta():
    delta = Group(
        base_joint([0,0,0,0,0,0]),
        base_link([0,0,0,0,0,0], Cylinder(radius=0.03,length=0.01), "Grey"),
        # linear actuator
        motor_joint(1, "prismatic", [0,0,0,0,0,0], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(1, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BrightBlue"),

        # # rotary actuator 2
        # motor_joint(2, "revolute", [0.005*np.sqrt(3), -0.005-0.03, 0.02, 0, 0, -np.radians(120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(2, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # # rotary actuator 3
        # motor_joint(3, "revolute", [-0.005*np.sqrt(3), -0.005-0.03, 0.02, 0, 0, np.radians(120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(3, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        #
        # # rotary actuator 4
        # motor_joint(4, "revolute", [0.005*np.sqrt(3)+0.03*np.sqrt(3)/2.0, -0.005+0.03/2.0, 0.02, 0, 0, -np.radians(120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(4, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # # rotary actuator 5
        # motor_joint(5, "revolute", [0.03*np.sqrt(3)/2.0, 0.01+0.03/2.0, 0.02, 0, 0, np.radians(0)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(5, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        #
        # # rotary actuator 6
        # motor_joint(6, "revolute", [-0.03*np.sqrt(3)/2.0, 0.01+0.03/2.0, 0.02, 0, 0, np.radians(0)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(6, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # # rotary actuator 7
        # motor_joint(7, "revolute", [-0.005*np.sqrt(3)-0.03*np.sqrt(3)/2.0, -0.005+0.03/2.0, 0.02, 0, 0, np.radians(120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(7, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),


        # rotary actuator 2
        motor_joint(2, "prismatic", [0.01*np.sqrt(3), -0.01-0.03, 0.0, 0, 0, -np.radians(120)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(2, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 3
        motor_joint(3, "prismatic", [-0.01*np.sqrt(3), -0.01-0.03, 0.0, 0, 0, np.radians(120)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(3, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # rotary actuator 4
        motor_joint(4, "prismatic", [0.01*np.sqrt(3)+0.03*np.sqrt(3)/2.0, -0.01+0.03/2.0, 0.0, 0, 0, -np.radians(120)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(4, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 5
        motor_joint(5, "prismatic", [0.03*np.sqrt(3)/2.0, 0.02+0.03/2.0, 0.0, 0, 0, np.radians(0)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(5, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # rotary actuator 6
        motor_joint(6, "prismatic", [-0.03*np.sqrt(3)/2.0, 0.02+0.03/2.0, 0.0, 0, 0, np.radians(0)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(6, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 7
        motor_joint(7, "revolute", [-0.01*np.sqrt(3)-0.03*np.sqrt(3)/2.0, -0.01+0.03/2.0, 0.0, 0, 0, np.radians(120)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(7, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # add bars on top of linear actuator
        bar_joint("bar_joint_1", "motor_link_1", "bar_link_1", [0,0,0.02,np.radians(90),0,0]),
        bar_link("bar_link_1", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        bar_joint("bar_joint_2", "motor_link_1", "bar_link_2", [0,0,0.02,np.radians(90),0,np.radians(120)]),
        bar_link("bar_link_2", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),


        bar_joint("bar_joint_3", "motor_link_1", "bar_link_3", [0,0,0.02,np.radians(90),0,-np.radians(120)]),
        bar_link("bar_link_3", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        # parallel leg on linear actuator
        parallel_leg(1, [0,0,0.02,-np.radians(90),0,0], 'bar_link_1'),
        parallel_leg(8, [0,0,0.02,-np.radians(90),0,0], 'bar_link_2'),
        parallel_leg(9, [0,0,0.02,-np.radians(90),0,0], 'bar_link_3'),
        # parallel leg on rotary actuator
        parallel_leg(2, [0,0,0.02,0,0,0], 'motor_link_2'),
        parallel_leg(3, [0,0,0.02,0,0,0], 'motor_link_3'),
        parallel_leg(4, [0,0,0.02,0,0,0], 'motor_link_4'),
        parallel_leg(5, [0,0,0.02,0,0,0], 'motor_link_5'),
        parallel_leg(6, [0,0,0.02,0,0,0], 'motor_link_6'),
        parallel_leg(7, [0,0,0.02,0,0,0], 'motor_link_7'),
    )
    return delta

def quadDelta():
    # all the inner actuators are shared between 2 legs from the same delta
    base_dis = 0.02
    delta = Group(
        base_joint([0,0,0,0,0,0]),
        base_link([0,0,0,0,0,0], Cylinder(radius=0.03,length=0.01), "Grey"),
        # linear actuator
        motor_joint(1, "prismatic", [0,0,0,0,0,0], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(1, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BrightBlue"),

        # rotary actuator 2
        motor_joint(2, "revolute", [0.0, -1*(0.02+base_dis), 0.02, 0, 0, -np.radians(180)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(2, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 3
        motor_joint(3, "revolute", [0.02+base_dis, 0.0, 0.02, 0, 0, -np.radians(90)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(3, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 4
        motor_joint(4, "revolute", [0.0, 0.02+base_dis, 0.02, 0, 0, np.radians(0)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(4, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 5
        motor_joint(5, "revolute", [-1*(0.02+base_dis), 0.0, 0.02, 0, 0, np.radians(90)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(5, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # add bars on top of linear actuator
        bar_joint("bar_joint_1", "motor_link_1", "bar_link_1", [0,0,0.02,np.radians(90),0,np.radians(0)]),
        bar_link("bar_link_1", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        bar_joint("bar_joint_2", "motor_link_1", "bar_link_2", [0,0,0.02,np.radians(90),0,np.radians(90)]),
        bar_link("bar_link_2", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        bar_joint("bar_joint_3", "motor_link_1", "bar_link_3", [0,0,0.02,np.radians(90),0,np.radians(180)]),
        bar_link("bar_link_3", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        bar_joint("bar_joint_4", "motor_link_1", "bar_link_4", [0,0,0.02,np.radians(90),0,-np.radians(90)]),
        bar_link("bar_link_4", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        # 2 legs sharing one actuator
        bar_joint("bar_joint_11", "bar_link_1", "bar_link_11", [0,0,0.02,0,np.radians(90),np.radians(0)]),
        bar_link("bar_link_11", [0,0,0.0,0,0,0], Cylinder(radius=0.001,length=0.01*np.sqrt(3)), "Grey"),
        parallel_leg(6, [0,0,-0.01*np.sqrt(3)/2,-np.radians(90),-np.radians(30),0], 'bar_link_11'),
        parallel_leg(7, [0,0,0.01*np.sqrt(3)/2,-np.radians(90),-np.radians(150),0], 'bar_link_11'),

        bar_joint("bar_joint_21", "bar_link_2", "bar_link_21", [0,0,0.02,0,np.radians(90),np.radians(0)]),
        bar_link("bar_link_21", [0,0,0.0,0,0,0], Cylinder(radius=0.001,length=0.01*np.sqrt(3)), "Grey"),
        parallel_leg(8, [0,0,-0.01*np.sqrt(3)/2,-np.radians(90),-np.radians(30),0], 'bar_link_21'),
        parallel_leg(9, [0,0,0.01*np.sqrt(3)/2,-np.radians(90),-np.radians(150),0], 'bar_link_21'),

        bar_joint("bar_joint_31", "bar_link_3", "bar_link_31", [0,0,0.02,0,np.radians(90),np.radians(0)]),
        bar_link("bar_link_31", [0,0,0.0,0,0,0], Cylinder(radius=0.001,length=0.01*np.sqrt(3)), "Grey"),
        parallel_leg(10, [0,0,-0.01*np.sqrt(3)/2,-np.radians(90),-np.radians(30),0], 'bar_link_31'),
        parallel_leg(11, [0,0,0.01*np.sqrt(3)/2,-np.radians(90),-np.radians(150),0], 'bar_link_31'),

        bar_joint("bar_joint_41", "bar_link_4", "bar_link_41", [0,0,0.02,0,np.radians(90),np.radians(0)]),
        bar_link("bar_link_41", [0,0,0.0,0,0,0], Cylinder(radius=0.001,length=0.01*np.sqrt(3)), "Grey"),
        parallel_leg(12, [0,0,-0.01*np.sqrt(3)/2,-np.radians(90),-np.radians(30),0], 'bar_link_41'),
        parallel_leg(13, [0,0,0.01*np.sqrt(3)/2,-np.radians(90),-np.radians(150),0], 'bar_link_41'),

        # parallel leg on rotary actuator
        parallel_leg(2, [0,0,0.02,0,0,0], 'motor_link_2'),
        parallel_leg(3, [0,0,0.02,0,0,0], 'motor_link_3'),
        parallel_leg(4, [0,0,0.02,0,0,0], 'motor_link_4'),
        parallel_leg(5, [0,0,0.02,0,0,0], 'motor_link_5'),
    )
    return delta

def quad7Delta():
    # 2 are 2 dof, 2 are 3dof
    base_dis = 0.02
    delta_dis = 0.01
    delta = Group(
        base_joint([0,0,0,0,0,0]),
        base_link([0,0,0,0,0,0], Cylinder(radius=0.03,length=0.01), "Grey"),
        # linear actuator
        motor_joint(1, "prismatic", [0,0,0,0,0,0], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(1, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BrightBlue"),

        # rotary actuator 2
        motor_joint(2, "revolute", [delta_dis*np.sqrt(3)/2, -delta_dis*1.5-base_dis, 0.02, 0, 0, -np.radians(120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(2, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 3
        motor_joint(3, "revolute", [-delta_dis*np.sqrt(3)/2, -delta_dis*1.5-base_dis, 0.02, 0, 0, np.radians(120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(3, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # rotary actuator 4
        motor_joint(4, "prismatic", [delta_dis*1.5+base_dis, 0.0, 0.0, 0, 0, -np.radians(90)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(4, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # rotary actuator 5
        motor_joint(5, "revolute", [-delta_dis*np.sqrt(3)/2, delta_dis*1.5+base_dis, 0.02, 0, 0, np.radians(60)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(5, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 6
        motor_joint(6, "revolute", [delta_dis*np.sqrt(3)/2, delta_dis*1.5+base_dis, 0.02, 0, 0, -np.radians(60)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(6, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # rotary actuator 7
        motor_joint(7, "prismatic", [-1*(delta_dis*1.5+base_dis), 0.0, 0.0, 0, 0, np.radians(90)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(7, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # add bars on top of linear actuator
        bar_joint("bar_joint_1", "motor_link_1", "bar_link_1", [0,0,0.02,np.radians(90),0,np.radians(0)]),
        bar_link("bar_link_1", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        bar_joint("bar_joint_2", "motor_link_1", "bar_link_2", [0,0,0.02,np.radians(90),0,np.radians(90)]),
        bar_link("bar_link_2", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        bar_joint("bar_joint_3", "motor_link_1", "bar_link_3", [0,0,0.02,np.radians(90),0,np.radians(180)]),
        bar_link("bar_link_3", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        bar_joint("bar_joint_4", "motor_link_1", "bar_link_4", [0,0,0.02,np.radians(90),0,-np.radians(90)]),
        bar_link("bar_link_4", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        # parallel leg on linear actuator
        parallel_leg(11, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_1'),
        parallel_leg(12, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_2'),
        parallel_leg(13, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_3'),
        parallel_leg(14, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_4'),

        # 2 legs sharing one actuator
        bar_joint("bar_joint_41", "motor_link_4", "bar_link_41", [0,0,0.02,np.radians(90),0,np.radians(90)]),
        bar_link("bar_link_41", [0,0,0.0,0,0,0], Cylinder(radius=0.001,length=0.01*np.sqrt(3)), "Grey"),
        parallel_leg(41, [0,0,-0.01*np.sqrt(3)/2,-np.radians(90),-np.radians(30),0], 'bar_link_41'),
        parallel_leg(42, [0,0,0.01*np.sqrt(3)/2,-np.radians(90),-np.radians(150),0], 'bar_link_41'),

        bar_joint("bar_joint_71", "motor_link_7", "bar_link_71", [0,0,0.02,np.radians(90),0,np.radians(90)]),
        bar_link("bar_link_71", [0,0,0.0,0,0,0], Cylinder(radius=0.001,length=0.01*np.sqrt(3)), "Grey"),
        parallel_leg(71, [0,0,-0.01*np.sqrt(3)/2,-np.radians(90),-np.radians(30),0], 'bar_link_71'),
        parallel_leg(72, [0,0,0.01*np.sqrt(3)/2,-np.radians(90),-np.radians(150),0], 'bar_link_71'),

        # parallel leg on rotary actuator
        parallel_leg(2, [0,0,0.02,0,0,0], 'motor_link_2'),
        parallel_leg(3, [0,0,0.02,0,0,0], 'motor_link_3'),
        parallel_leg(5, [0,0,0.02,0,0,0], 'motor_link_5'),
        parallel_leg(6, [0,0,0.02,0,0,0], 'motor_link_6'),
    )
    return delta


def quad5Delta():
    # 2 are 2 dof, 2 are 3dof
    base_dis = 0.02
    delta_dis = 0.02
    motor_dis = base_dis/np.sqrt(2) + delta_dis*np.sqrt(3)*np.cos(np.radians(15))
    bar_dis = base_dis/np.sqrt(2) + delta_dis*np.sqrt(3)*np.sin(np.radians(15))

    delta = Group(
        base_joint([0,0,0,0,0,0]),
        base_link([0,0,0,0,0,0], Cylinder(radius=0.03,length=0.01), "Grey"),
        # linear actuator
        motor_joint(1, "prismatic", [0,0,0,0,0,0], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(1, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BrightBlue"),

        # actuator 2
        motor_joint(2, "prismatic", [motor_dis/np.sqrt(2), -motor_dis/np.sqrt(2), 0.0, 0, 0, np.radians(-135)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(2, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # actuator 3
        motor_joint(3, "prismatic", [motor_dis/np.sqrt(2), motor_dis/np.sqrt(2), 0.0, 0, 0, np.radians(-45)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(3, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # actuator 4
        motor_joint(4, "prismatic", [-motor_dis/np.sqrt(2), motor_dis/np.sqrt(2), 0.0, 0, 0, np.radians(45)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(4, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # actuator 5
        motor_joint(5, "prismatic", [-motor_dis/np.sqrt(2), -motor_dis/np.sqrt(2), 0.0, 0, 0, np.radians(135)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(5, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),


        # add bars on top of linear actuator
        bar_joint("bar_joint_1", "motor_link_1", "bar_link_1", [0,0,0.02,np.radians(90),0,np.radians(0)]),
        bar_link("bar_link_1", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        bar_joint("bar_joint_2", "motor_link_1", "bar_link_2", [0,0,0.02,np.radians(90),0,np.radians(90)]),
        bar_link("bar_link_2", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        bar_joint("bar_joint_3", "motor_link_1", "bar_link_3", [0,0,0.02,np.radians(90),0,np.radians(180)]),
        bar_link("bar_link_3", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        bar_joint("bar_joint_4", "motor_link_1", "bar_link_4", [0,0,0.02,np.radians(90),0,-np.radians(90)]),
        bar_link("bar_link_4", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        # parallel leg on linear actuator
        parallel_leg(11, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_1'),
        parallel_leg(12, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_2'),
        parallel_leg(13, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_3'),
        parallel_leg(14, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_4'),

        # 2 legs sharing one actuator
        bar_joint("bar_joint_21", "motor_link_2", "bar_link_21", [0,0,0.02,np.radians(90),0,np.radians(90)]),
        bar_link("bar_link_21", [0,0,0.0,0,0,0], Cylinder(radius=0.001,length=bar_dis*2), "Grey"),
        parallel_leg(21, [0,0,-bar_dis,-np.radians(90),np.radians(-105),0], 'bar_link_21'),
        parallel_leg(22, [0,0,bar_dis,-np.radians(90),np.radians(-75),0], 'bar_link_21'),

        bar_joint("bar_joint_31", "motor_link_3", "bar_link_31", [0,0,0.02,np.radians(90),0,np.radians(90)]),
        bar_link("bar_link_31", [0,0,0.0,0,0,0], Cylinder(radius=0.001,length=bar_dis*2), "Grey"),
        parallel_leg(31, [0,0,-bar_dis,-np.radians(90),np.radians(-105),0], 'bar_link_31'),
        parallel_leg(32, [0,0,bar_dis,-np.radians(90),np.radians(-75),0], 'bar_link_31'),

        bar_joint("bar_joint_41", "motor_link_4", "bar_link_41", [0,0,0.02,np.radians(90),0,np.radians(90)]),
        bar_link("bar_link_41", [0,0,0.0,0,0,0], Cylinder(radius=0.001,length=bar_dis*2), "Grey"),
        parallel_leg(41, [0,0,-bar_dis,-np.radians(90),np.radians(-105),0], 'bar_link_41'),
        parallel_leg(42, [0,0,bar_dis,-np.radians(90),np.radians(-75),0], 'bar_link_41'),

        bar_joint("bar_joint_51", "motor_link_5", "bar_link_51", [0,0,0.02,np.radians(90),0,np.radians(90)]),
        bar_link("bar_link_51", [0,0,0.0,0,0,0], Cylinder(radius=0.001,length=bar_dis*2), "Grey"),
        parallel_leg(51, [0,0,-bar_dis,-np.radians(90),np.radians(-105),0], 'bar_link_51'),
        parallel_leg(52, [0,0,bar_dis,-np.radians(90),np.radians(-75),0], 'bar_link_51'),

    )
    return delta

def quad8Delta():
    # 2 are 2 dof, 2 are 3dof
    base_dis = 0.02
    delta_dis = 0.02
    motor_dis = base_dis*np.sqrt(2) - (base_dis-delta_dis*np.sqrt(3)/2)/np.sqrt(2)
    bar_dis = (base_dis-delta_dis*np.sqrt(3)/2)/np.sqrt(2)

    delta = Group(
        base_joint([0,0,0,0,0,0]),
        base_link([0,0,0,0,0,0], Cylinder(radius=0.03,length=0.01), "Grey"),
        # rotary actuator 1
        motor_joint(1, "revolute", [0.0, -(delta_dis*1.5+base_dis), 0.02, 0, 0, np.radians(-180)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(1, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # rotary actuator 2
        motor_joint(2, "revolute", [(delta_dis*1.5+base_dis), 0.0, 0.02, 0, 0, np.radians(-90)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(2, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # rotary actuator 3
        motor_joint(3, "revolute", [0.0, (delta_dis*1.5+base_dis), 0.02, 0, 0, np.radians(0)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(3, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # rotary actuator 4
        motor_joint(4, "revolute", [-(delta_dis*1.5+base_dis), 0.0, 0.02, 0, 0, np.radians(90)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(4, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),


        # linear actuator 5
        motor_joint(5, "prismatic", [motor_dis/np.sqrt(2), -motor_dis/np.sqrt(2), 0.0, 0, 0, np.radians(45)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(5, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # linear actuator 6
        motor_joint(6, "prismatic", [motor_dis/np.sqrt(2), motor_dis/np.sqrt(2), 0.0, 0, 0, np.radians(135)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(6, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # linear actuator 7
        motor_joint(7, "prismatic", [-motor_dis/np.sqrt(2), motor_dis/np.sqrt(2), 0.0, 0, 0, np.radians(-135)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(7, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # linear actuator 8
        motor_joint(8, "prismatic", [-motor_dis/np.sqrt(2), -motor_dis/np.sqrt(2), 0.0, 0, 0, np.radians(-45)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(8, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),


        # 2 legs sharing one actuator
        bar_joint("bar_joint_51", "motor_link_5", "bar_link_51", [0,0,0.02,np.radians(90),0,np.radians(90)]),
        bar_link("bar_link_51", [0,0,0.0,0,0,0], Cylinder(radius=0.001,length=bar_dis*2), "Grey"),
        parallel_leg(51, [0,0,-bar_dis,-np.radians(90),np.radians(165),0], 'bar_link_51'),
        parallel_leg(52, [0,0,bar_dis,-np.radians(90),np.radians(15),0], 'bar_link_51'),

        bar_joint("bar_joint_61", "motor_link_6", "bar_link_61", [0,0,0.02,np.radians(90),0,np.radians(90)]),
        bar_link("bar_link_61", [0,0,0.0,0,0,0], Cylinder(radius=0.001,length=bar_dis*2), "Grey"),
        parallel_leg(61, [0,0,-bar_dis,-np.radians(90),np.radians(165),0], 'bar_link_61'),
        parallel_leg(62, [0,0,bar_dis,-np.radians(90),np.radians(15),0], 'bar_link_61'),

        bar_joint("bar_joint_71", "motor_link_7", "bar_link_71", [0,0,0.02,np.radians(90),0,np.radians(90)]),
        bar_link("bar_link_71", [0,0,0.0,0,0,0], Cylinder(radius=0.001,length=bar_dis*2), "Grey"),
        parallel_leg(71, [0,0,-bar_dis,-np.radians(90),np.radians(165),0], 'bar_link_71'),
        parallel_leg(72, [0,0,bar_dis,-np.radians(90),np.radians(15),0], 'bar_link_71'),

        bar_joint("bar_joint_81", "motor_link_8", "bar_link_81", [0,0,0.02,np.radians(90),0,np.radians(90)]),
        bar_link("bar_link_81", [0,0,0.0,0,0,0], Cylinder(radius=0.001,length=bar_dis*2), "Grey"),
        parallel_leg(81, [0,0,-bar_dis,-np.radians(90),np.radians(165),0], 'bar_link_81'),
        parallel_leg(82, [0,0,bar_dis,-np.radians(90),np.radians(15),0], 'bar_link_81'),

        # parallel leg on rotary actuator
        parallel_leg(1, [0,0,0.02,0,0,0], 'motor_link_1'),
        parallel_leg(2, [0,0,0.02,0,0,0], 'motor_link_2'),
        parallel_leg(3, [0,0,0.02,0,0,0], 'motor_link_3'),
        parallel_leg(4, [0,0,0.02,0,0,0], 'motor_link_4'),

    )
    return delta

def starDelta():
    # all the outside actuators are shared between 2 legs from 2 different delta
    delta = Group(
        base_joint([0,0,0,0,0,0]),
        base_link([0,0,0,0,0,0], Cylinder(radius=0.03,length=0.01), "Grey"),
        # linear actuator
        motor_joint(1, "prismatic", [0,0,0,0,0,0], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(1, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BrightBlue"),

        # rotary actuator 2
        motor_joint(2, "revolute", [0.0, -1*(0.01*np.sqrt(3)+0.02), 0.02, 0, 0, -np.radians(180)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(2, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 3
        motor_joint(3, "revolute", [0.01*np.sqrt(3)*np.sqrt(3)/2.0+0.02*np.sqrt(3)/2.0, -1*(0.01*np.sqrt(3)/2.0+0.02/2.0), 0.02, 0, 0, -np.radians(120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(3, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 4
        motor_joint(4, "revolute", [0.01*np.sqrt(3)*np.sqrt(3)/2.0+0.02*np.sqrt(3)/2.0, 0.01*np.sqrt(3)/2.0+0.02/2.0, 0.02, 0, 0, -np.radians(60)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(4, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 5
        motor_joint(5, "revolute", [0.0, 0.01*np.sqrt(3)+0.02, 0.02, 0, 0, np.radians(0)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(5, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 6
        motor_joint(6, "revolute", [-1*(0.01*np.sqrt(3)*np.sqrt(3)/2.0+0.02*np.sqrt(3)/2.0), 0.01*np.sqrt(3)/2.0+0.02/2.0, 0.02, 0, 0, np.radians(60)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(6, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 7
        motor_joint(7, "revolute", [-1*(0.01*np.sqrt(3)*np.sqrt(3)/2.0+0.02*np.sqrt(3)/2.0), -1*(0.01*np.sqrt(3)/2.0+0.02/2.0), 0.02, 0, 0, np.radians(120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(7, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # add bars on top of linear actuator
        bar_joint("bar_joint_1", "motor_link_1", "bar_link_1", [0,0,0.02,np.radians(90),0,np.radians(30)]),
        bar_link("bar_link_1", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        bar_joint("bar_joint_2", "motor_link_1", "bar_link_2", [0,0,0.02,np.radians(90),0,np.radians(30+60)]),
        bar_link("bar_link_2", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        bar_joint("bar_joint_3", "motor_link_1", "bar_link_3", [0,0,0.02,np.radians(90),0,np.radians(30+60*2)]),
        bar_link("bar_link_3", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        bar_joint("bar_joint_4", "motor_link_1", "bar_link_4", [0,0,0.02,np.radians(90),0,-np.radians(30+60*2)]),
        bar_link("bar_link_4", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        bar_joint("bar_joint_5", "motor_link_1", "bar_link_5", [0,0,0.02,np.radians(90),0,-np.radians(30+60)]),
        bar_link("bar_link_5", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        bar_joint("bar_joint_6", "motor_link_1", "bar_link_6", [0,0,0.02,np.radians(90),0,-np.radians(30)]),
        bar_link("bar_link_6", [0,0,0.01,0,0,0], Cylinder(radius=0.001,length=0.02), "Grey"),

        # parallel leg on linear actuator
        parallel_leg(1, [0,0,0.02,-np.radians(90),0,0], 'bar_link_1'),
        parallel_leg(2, [0,0,0.02,-np.radians(90),0,0], 'bar_link_2'),
        parallel_leg(3, [0,0,0.02,-np.radians(90),0,0], 'bar_link_3'),
        parallel_leg(4, [0,0,0.02,-np.radians(90),0,0], 'bar_link_4'),
        parallel_leg(5, [0,0,0.02,-np.radians(90),0,0], 'bar_link_5'),
        parallel_leg(6, [0,0,0.02,-np.radians(90),0,0], 'bar_link_6'),

        # parallel leg on rotary actuator
        double_parallel_leg(7, [0,0,0.02,0,0,0], 'motor_link_2'),
        double_parallel_leg(8, [0,0,0.02,0,0,0], 'motor_link_3'),
        double_parallel_leg(9, [0,0,0.02,0,0,0], 'motor_link_4'),
        double_parallel_leg(10, [0,0,0.02,0,0,0], 'motor_link_5'),
        double_parallel_leg(11, [0,0,0.02,0,0,0], 'motor_link_6'),
        double_parallel_leg(12, [0,0,0.02,0,0,0], 'motor_link_7'),
    )
    return delta

def quadDeltaS():
    # each delta will have its own outside actuators
    base_dis = 0.03
    delta_dis = 0.02
    delta = Group(
        base_joint([0,0,0,0,0,0]),
        base_link([0,0,0,0,0,0], Cylinder(radius=0.03,length=0.01), "Grey"),
        # linear actuator
        motor_joint(1, "prismatic", [0,0,0,0,0,0], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(1, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BrightBlue"),

        # # rotary actuator 2
        # motor_joint(2, "revolute", [delta_dis*np.sqrt(3)/2, -delta_dis*1.5-base_dis, 0.02, 0, 0, -np.radians(120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(2, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # # rotary actuator 3
        # motor_joint(3, "revolute", [-delta_dis*np.sqrt(3)/2, -delta_dis*1.5-base_dis, 0.02, 0, 0, np.radians(120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(3, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # # rotary actuator 4
        # motor_joint(4, "revolute", [delta_dis*1.5+base_dis, delta_dis*np.sqrt(3)/2, 0.02, 0, 0, -np.radians(30)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(4, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # # rotary actuator 5
        # motor_joint(5, "revolute", [delta_dis*1.5+base_dis, -delta_dis*np.sqrt(3)/2, 0.02, 0, 0, -np.radians(150)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(5, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # # rotary actuator 6
        # motor_joint(6, "revolute", [-delta_dis*np.sqrt(3)/2, delta_dis*1.5+base_dis, 0.02, 0, 0, np.radians(60)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(6, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # # rotary actuator 7
        # motor_joint(7, "revolute", [delta_dis*np.sqrt(3)/2, delta_dis*1.5+base_dis, 0.02, 0, 0, -np.radians(60)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(7, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # # rotary actuator 8
        # motor_joint(8, "revolute", [-delta_dis*1.5-base_dis, -delta_dis*np.sqrt(3)/2, 0.02, 0, 0, np.radians(150)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(8, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # # rotary actuator 9
        # motor_joint(9, "revolute", [-delta_dis*1.5-base_dis, delta_dis*np.sqrt(3)/2, 0.02, 0, 0, np.radians(30)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        # motor_link(9, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        #  actuator 2
        motor_joint(2, "prismatic", [delta_dis*np.sqrt(3)/2, -delta_dis*1.5-base_dis, 0.0, 0, 0, -np.radians(120)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(2, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        #  actuator 3
        motor_joint(3, "prismatic", [-delta_dis*np.sqrt(3)/2, -delta_dis*1.5-base_dis, 0.0, 0, 0, np.radians(120)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(3, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        #  actuator 4
        motor_joint(4, "prismatic", [delta_dis*1.5+base_dis, delta_dis*np.sqrt(3)/2, 0.0, 0, 0, -np.radians(30)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(4, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        #  actuator 5
        motor_joint(5, "prismatic", [delta_dis*1.5+base_dis, -delta_dis*np.sqrt(3)/2, 0.0, 0, 0, -np.radians(150)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(5, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        #  actuator 6
        motor_joint(6, "prismatic", [-delta_dis*np.sqrt(3)/2, delta_dis*1.5+base_dis, 0.0, 0, 0, np.radians(60)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(6, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        #  actuator 7
        motor_joint(7, "prismatic", [delta_dis*np.sqrt(3)/2, delta_dis*1.5+base_dis, 0.0, 0, 0, -np.radians(60)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(7, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        #  actuator 8
        motor_joint(8, "prismatic", [-delta_dis*1.5-base_dis, -delta_dis*np.sqrt(3)/2, 0.0, 0, 0, np.radians(150)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(8, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        #  actuator 9
        motor_joint(9, "prismatic", [-delta_dis*1.5-base_dis, delta_dis*np.sqrt(3)/2, 0.0, 0, 0, np.radians(30)], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(9, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # add bars on top of linear actuator
        bar_joint("bar_joint_1", "motor_link_1", "bar_link_1", [0,0,0.02,np.radians(90),0,np.radians(0)]),
        bar_link("bar_link_1", [0,0,base_dis/2,0,0,0], Cylinder(radius=0.001,length=base_dis), "Grey"),

        bar_joint("bar_joint_2", "motor_link_1", "bar_link_2", [0,0,0.02,np.radians(90),0,np.radians(90)]),
        bar_link("bar_link_2", [0,0,base_dis/2,0,0,0], Cylinder(radius=0.001,length=base_dis), "Grey"),

        bar_joint("bar_joint_3", "motor_link_1", "bar_link_3", [0,0,0.02,np.radians(90),0,np.radians(180)]),
        bar_link("bar_link_3", [0,0,base_dis/2,0,0,0], Cylinder(radius=0.001,length=base_dis), "Grey"),

        bar_joint("bar_joint_4", "motor_link_1", "bar_link_4", [0,0,0.02,np.radians(90),0,-np.radians(90)]),
        bar_link("bar_link_4", [0,0,base_dis/2,0,0,0], Cylinder(radius=0.001,length=base_dis), "Grey"),

        # parallel leg on linear actuator
        parallel_leg(1, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_1'),
        parallel_leg(2, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_2'),
        parallel_leg(3, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_3'),
        parallel_leg(4, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_4'),

        # parallel leg on rotary actuator
        parallel_leg(5, [0,0,0.02,0,0,0], 'motor_link_2'),
        parallel_leg(6, [0,0,0.02,0,0,0], 'motor_link_3'),
        parallel_leg(7, [0,0,0.02,0,0,0], 'motor_link_4'),
        parallel_leg(8, [0,0,0.02,0,0,0], 'motor_link_5'),
        parallel_leg(9, [0,0,0.02,0,0,0], 'motor_link_6'),
        parallel_leg(10, [0,0,0.02,0,0,0], 'motor_link_7'),
        parallel_leg(11, [0,0,0.02,0,0,0], 'motor_link_8'),
        parallel_leg(12, [0,0,0.02,0,0,0], 'motor_link_9'),
    )
    return delta

def starDeltaS():
    # each delta will have its own outside actuators
    base_dis = 0.04
    delta_dis = 0.01
    delta = Group(
        base_joint([0,0,0,0,0,0]),
        base_link([0,0,0,0,0,0], Cylinder(radius=0.03,length=0.01), "Grey"),
        # linear actuator
        motor_joint(1, "prismatic", [0,0,0,0,0,0], limit=[0.0,0.1], axis=[0,0,1]),
        motor_link(1, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BrightBlue"),

        # rotary actuator 2
        motor_joint(2, "revolute", [delta_dis*np.sqrt(3)/2, -delta_dis*1.5-base_dis, 0.02, 0, 0, np.radians(-120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(2, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 3
        motor_joint(3, "revolute", [-delta_dis*np.sqrt(3)/2, -delta_dis*1.5-base_dis, 0.02, 0, 0, np.radians(120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(3, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 4
        motor_joint(4, "revolute", [(base_dis+delta_dis)*np.sqrt(3)/2+delta_dis*np.sqrt(3)/2, -(base_dis+delta_dis)/2+delta_dis/2, 0.02, 0, 0, np.radians(-60)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(4, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 5
        motor_joint(5, "revolute", [(base_dis+delta_dis)*np.sqrt(3)/2, -(base_dis+delta_dis)/2-delta_dis, 0.02, 0, 0, np.radians(-180)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(5, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 6
        motor_joint(6, "revolute", [(base_dis+delta_dis)*np.sqrt(3)/2, (base_dis+delta_dis)/2+delta_dis, 0.02, 0, 0, np.radians(0)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(6, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 7
        motor_joint(7, "revolute", [(base_dis+delta_dis)*np.sqrt(3)/2+delta_dis*np.sqrt(3)/2, (base_dis+delta_dis)/2-delta_dis/2, 0.02, 0, 0, np.radians(-120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(7, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 8
        motor_joint(8, "revolute", [-delta_dis*np.sqrt(3)/2, delta_dis*1.5+base_dis, 0.02, 0, 0, np.radians(60)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(8, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 9
        motor_joint(9, "revolute", [delta_dis*np.sqrt(3)/2, delta_dis*1.5+base_dis, 0.02, 0, 0, np.radians(-60)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(9, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 10
        motor_joint(10, "revolute", [-(base_dis+delta_dis)*np.sqrt(3)/2-delta_dis*np.sqrt(3)/2, (base_dis+delta_dis)/2-delta_dis/2, 0.02, 0, 0, np.radians(120)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(10, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 11
        motor_joint(11, "revolute", [-(base_dis+delta_dis)*np.sqrt(3)/2, (base_dis+delta_dis)/2+delta_dis, 0.02, 0, 0, np.radians(0)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(11, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 12
        motor_joint(12, "revolute", [-(base_dis+delta_dis)*np.sqrt(3)/2, -(base_dis+delta_dis)/2-delta_dis, 0.02, 0, 0, np.radians(180)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(12, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),
        # rotary actuator 13
        motor_joint(13, "revolute", [-(base_dis+delta_dis)*np.sqrt(3)/2-delta_dis*np.sqrt(3)/2, -(base_dis+delta_dis)/2+delta_dis/2, 0.02, 0, 0, np.radians(60)], limit=[-0.5*np.pi,0.0], axis=[1,0,0]),
        motor_link(13, [0,0,0.01,0,0,0], Box(size=[0.01, 0.002, 0.02]), "BananaYellow"),

        # add bars on top of linear actuator
        bar_joint("bar_joint_1", "motor_link_1", "bar_link_1", [0,0,0.02,np.radians(90),0,np.radians(0)]),
        bar_link("bar_link_1", [0,0,base_dis/2,0,0,0], Cylinder(radius=0.001,length=base_dis), "Grey"),

        bar_joint("bar_joint_2", "motor_link_1", "bar_link_2", [0,0,0.02,np.radians(90),0,np.radians(60)]),
        bar_link("bar_link_2", [0,0,base_dis/2,0,0,0], Cylinder(radius=0.001,length=base_dis), "Grey"),

        bar_joint("bar_joint_3", "motor_link_1", "bar_link_3", [0,0,0.02,np.radians(90),0,np.radians(60*2)]),
        bar_link("bar_link_3", [0,0,base_dis/2,0,0,0], Cylinder(radius=0.001,length=base_dis), "Grey"),

        bar_joint("bar_joint_4", "motor_link_1", "bar_link_4", [0,0,0.02,np.radians(90),0,np.radians(60*3)]),
        bar_link("bar_link_4", [0,0,base_dis/2,0,0,0], Cylinder(radius=0.001,length=base_dis), "Grey"),

        bar_joint("bar_joint_5", "motor_link_1", "bar_link_5", [0,0,0.02,np.radians(90),0,np.radians(-60*2)]),
        bar_link("bar_link_5", [0,0,base_dis/2,0,0,0], Cylinder(radius=0.001,length=base_dis), "Grey"),

        bar_joint("bar_joint_6", "motor_link_1", "bar_link_6", [0,0,0.02,np.radians(90),0,np.radians(-60)]),
        bar_link("bar_link_6", [0,0,base_dis/2,0,0,0], Cylinder(radius=0.001,length=base_dis), "Grey"),

        # parallel leg on linear actuator
        parallel_leg(1, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_1'),
        parallel_leg(2, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_2'),
        parallel_leg(3, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_3'),
        parallel_leg(4, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_4'),
        parallel_leg(5, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_5'),
        parallel_leg(6, [0,0,base_dis,-np.radians(90),0,0], 'bar_link_6'),

        # parallel leg on rotary actuator
        parallel_leg(7, [0,0,0.02,0,0,0], 'motor_link_2'),
        parallel_leg(8, [0,0,0.02,0,0,0], 'motor_link_3'),
        parallel_leg(9, [0,0,0.02,0,0,0], 'motor_link_4'),
        parallel_leg(10, [0,0,0.02,0,0,0], 'motor_link_5'),
        parallel_leg(11, [0,0,0.02,0,0,0], 'motor_link_6'),
        parallel_leg(12, [0,0,0.02,0,0,0], 'motor_link_7'),
        parallel_leg(13, [0,0,0.02,0,0,0], 'motor_link_8'),
        parallel_leg(14, [0,0,0.02,0,0,0], 'motor_link_9'),
        parallel_leg(15, [0,0,0.02,0,0,0], 'motor_link_10'),
        parallel_leg(16, [0,0,0.02,0,0,0], 'motor_link_11'),
        parallel_leg(17, [0,0,0.02,0,0,0], 'motor_link_12'),
        parallel_leg(18, [0,0,0.02,0,0,0], 'motor_link_13'),
    )
    return delta


if __name__ == "__main__":
    delta = Robot(
        delta_materials.materials,
        Link("world"),
        twinDelta(),
        name = "twinlinearDelta"
    )
    print(delta)
