from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import serial
import os
import threading
 
ESCAPE = '\033'
 
window = 0
 
#rotation
X_AXIS = 0.0
Y_AXIS = 0.0
Z_AXIS = 0.0
W_     = 0.0

m00, m01, m02, m10, m11, m12, m20, m21, m22 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
 
DIRECTION = 1

exit_sig = 0
 
 
def InitGL(Width, Height): 
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearColor(0.0, 0.4, 0.4, 0.0)

        glClearDepth(1.0) 
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        glShadeModel(GL_SMOOTH)   
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

 
def keyPressed(*args):
        if args[0] == ESCAPE:
                sys.exit()

 
def DrawGLScene():
        global X_AXIS,Y_AXIS,Z_AXIS
        global DIRECTION
        global exit_sig
        global m00, m01, m02, m10, m11, m12, m20, m21, m22
 
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
 
        glLoadIdentity()
        glTranslatef(0.0,0.0,-6.0)
 
        glRotatef(X_AXIS,1.0,0.0,0.0)
        glRotatef(Y_AXIS,0.0,1.0,0.0)
        glRotatef(Z_AXIS,0.0,0.0,1.0)
        #glRotatef((180 / 3.14), m00, m01, m02)
        #glRotatef((180 / 3.14), m10, m11, m12)
        #glRotatef((180 / 3.14), m20, m21, m22)

        #glRotatef(W_, X_AXIS,    0.0,    0.0)
        #glRotatef(W_,    0.0, Y_AXIS,    0.0)
        #glRotatef(W_,    0.0,    0.0, Z_AXIS)

 
        # Draw Cube (multiple quads)
        glBegin(GL_QUADS)
 
        glColor4f(0.0,1.0,0.0,0.1)
        glVertex3f( 1.0, 1.0,-1.0)
        glVertex3f(-1.0, 1.0,-1.0)
        glVertex3f(-1.0, 1.0, 1.0)
        glVertex3f( 1.0, 1.0, 1.0) 
 
        glColor4f(1.0,0.0,0.0,0.6)
        glVertex3f( 1.0,-1.0, 1.0)
        glVertex3f(-1.0,-1.0, 1.0)
        glVertex3f(-1.0,-1.0,-1.0)
        glVertex3f( 1.0,-1.0,-1.0) 
 
        glColor4f(0.0,1.0,1.0,0.6)
        glVertex3f( 1.0, 1.0, 1.0)
        glVertex3f(-1.0, 1.0, 1.0)
        glVertex3f(-1.0,-1.0, 1.0)
        glVertex3f( 1.0,-1.0, 1.0)
 
        glColor4f(1.0,1.0,0.0,0.6)
        glVertex3f( 1.0,-1.0,-1.0)
        glVertex3f(-1.0,-1.0,-1.0)
        glVertex3f(-1.0, 1.0,-1.0)
        glVertex3f( 1.0, 1.0,-1.0)
 
        glColor4f(0.0,0.0,1.0,0.6)
        glVertex3f(-1.0, 1.0, 1.0) 
        glVertex3f(-1.0, 1.0,-1.0)
        glVertex3f(-1.0,-1.0,-1.0) 
        glVertex3f(-1.0,-1.0, 1.0) 
 
        glColor4f(1.0,0.0,1.0,0.6)
        glVertex3f( 1.0, 1.0,-1.0) 
        glVertex3f( 1.0, 1.0, 1.0)
        glVertex3f( 1.0,-1.0, 1.0)
        glVertex3f( 1.0,-1.0,-1.0)

        glEnd()
 
 
        #X_AXIS = X_AXIS - 0.30
        #Z_AXIS = Z_AXIS - 0.30
        #Z_AXIS = 45
 
        glutSwapBuffers()


def close_(args):
        global exit_sig

        #print("TEST")
        DrawGLScene()

        if(exit_sig):
           exit()
        
        glutTimerFunc(1, close_, 0)


def gl_draw_():
 
        global window
 
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH ) #GL_BLEND_EQUATION_ALPHA)
        glutInitWindowSize(640,480)
        glutInitWindowPosition(200,200)

        window = glutCreateWindow('OpenGL Python Cube')
 
        glutDisplayFunc(DrawGLScene)

        glutTimerFunc(1, close_, 0)
        #glutIdleFunc(DrawGLScene)

        glutKeyboardFunc(keyPressed)
        InitGL(640, 480)
        glutMainLoop()
        print("RET2")
        return


#gl_draw_()
 
#if __name__ == "__main__":
#        main() 