from odio_urdf import *

materials=Group(
    Material("Black",Color(rgba="0 0 0 1")),
    Material("Blue",Color(rgba="0.0 0.0 0.8 1.0")),
    Material("Green",Color(rgba="0.0 0.8 0.0 1.0")),
    Material("Grey",Color(rgba="0.4 0.4 0.4 1.0")),
    Material("Orange",Color(rgba=str(255.0/255)+" "+
        str(108.0/255)+" "+str(10.0/255)+" 1.0")),
    Material("Brown",Color(rgba=str(222.0/255)+" "+
        str(207.0/255)+" "+str(195.0/255)+" 1.0")),
    Material("Red",Color(rgba="0.8 0.0 0.0 1.0")),
    Material("BananaYellow", Color(rgba="0.9803921568627451 0.996078431372549 0.29411764705882354 1.0")),
    Material("BrightBlue", Color(rgba="0.00392156862745098 0.396078431372549 0.9882352941176471 1.0")),
    Material("White",Color(rgba="1.0 1.0 1.0 1.0")))
