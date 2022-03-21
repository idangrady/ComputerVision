/*
 * Glut.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: Coert and a guy named Frank
 */

#include "Glut.h"

#ifdef __linux__
#include <GL/freeglut_std.h>
#endif
#include <GL/glu.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <stddef.h>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <valarray>
#include <vector>
#include <cstdlib>
#include <algorithm>
#include <cstdlib>

#include "../utilities/General.h"
#include "arcball.h"
#include "Camera.h"
#include "Reconstructor.h"
#include "Scene3DRenderer.h"

using namespace std;
using namespace cv;
using namespace cv::ml;

namespace nl_uu_science_gmt
{

	Glut* Glut::m_Glut;

	Glut::Glut(
		Scene3DRenderer& s3d) :
		m_scene3d(s3d)
	{
		// static pointer to this class so we can get to it from the static GL events
		m_Glut = this;
	}

	Glut::~Glut()
	{
	}

#ifdef __linux__
	/**
	 * Main OpenGL initialisation for Linux-like system (with Glut)
	 */
	void Glut::initializeLinux(
		const char* win_name, int argc, char** argv)
	{
		arcball_reset();	//initialize the ArcBall for scene rotation

		glutInit(&argc, argv);
		glutInitWindowSize(m_Glut->getScene3d().getWidth(), m_Glut->getScene3d().getHeight());
		glutInitWindowPosition(700, 10);
		glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

		glutCreateWindow(win_name);

		glutReshapeFunc(reshape);
		glutDisplayFunc(display);
		glutKeyboardFunc(keyboard);
		glutIdleFunc(idle);
		glutMouseFunc(mouse);
		glutMotionFunc(motion);

		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);

		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LESS);

		glutTimerFunc(10, update, 0);

		// from now on it's just events
		glutMainLoop();
	}
#elif defined _WIN32
	/**
	 * Main OpenGL initialisation for Windows-like system (without Glut)
	 */

	static bool initialized_coloring = false;
	static Mat initialized_data;

	static vector<vector<Point2f>> centers_history(4);

	static vector<vector<MatND>> HistBase(4);


	int Glut::initializeWindows(const char* win_name)
	{
		Scene3DRenderer& scene3d = m_Glut->getScene3d();
		arcball_reset();	//initialize the ArcBall for scene rotation

		WNDCLASSEX windowClass;//window class
		HWND hwnd;//window handle
		DWORD dwExStyle;//window extended style
		DWORD dwStyle;//window style
		RECT windowRect;

		/*      Screen/display attributes*/
		int width = scene3d.getWidth();
		int height = scene3d.getHeight();
		int bits = 32;


		windowRect.left = (long)0;               //set left value to 0
		windowRect.right = (long)width;//set right value to requested width
		windowRect.top = (long)0;//set top value to 0
		windowRect.bottom = (long)height;//set bottom value to requested height

		/*      Fill out the window class structure*/
		windowClass.cbSize = sizeof(WNDCLASSEX);
		windowClass.style = CS_HREDRAW | CS_VREDRAW;
		windowClass.lpfnWndProc = Glut::WndProc;
		windowClass.cbClsExtra = 0;
		windowClass.cbWndExtra = 0;
		windowClass.hInstance = 0;                //hInstance;
		windowClass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
		windowClass.hCursor = LoadCursor(NULL, IDC_ARROW);
		windowClass.hbrBackground = NULL;
		windowClass.lpszMenuName = NULL;
		windowClass.lpszClassName = LPCSTR("Glut");
		windowClass.hIconSm = LoadIcon(NULL, IDI_WINLOGO);

		/*      Register window class*/
		if (!RegisterClassEx(&windowClass))
		{
			return 0;
		}

		/*      Check if fullscreen is on*/
		if (scene3d.isShowFullscreen())
		{
			DEVMODE dmScreenSettings;
			memset(&dmScreenSettings, 0, sizeof(dmScreenSettings));
			dmScreenSettings.dmSize = sizeof(dmScreenSettings);
			dmScreenSettings.dmPelsWidth = width;   //screen width
			dmScreenSettings.dmPelsHeight = height;//screen height
			dmScreenSettings.dmBitsPerPel = bits;//bits per pixel
			dmScreenSettings.dmFields = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT;

			if (ChangeDisplaySettings(&dmScreenSettings, CDS_FULLSCREEN !=
				DISP_CHANGE_SUCCESSFUL))
			{
				/*      Setting display mode failed, switch to windowed*/
				MessageBox(NULL, LPCSTR("Display mode failed"), NULL, MB_OK);
				scene3d.setShowFullscreen(false);
			}
		}

		/*      Check if fullscreen is still on*/
		if (scene3d.isShowFullscreen())
		{
			dwExStyle = WS_EX_APPWINDOW;    //window extended style
			dwStyle = WS_POPUP;//windows style
			ShowCursor(FALSE);//hide mouse pointer
		}
		else
		{
			dwExStyle = WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;  //window extended style
			dwStyle = WS_OVERLAPPEDWINDOW;//windows style
		}

		AdjustWindowRectEx(&windowRect, dwStyle, FALSE, dwExStyle);

		/*      Class registerd, so now create our window*/
		hwnd = CreateWindowEx(NULL, LPCSTR("Glut"),  //class name
			LPCSTR(win_name),//app name
			dwStyle |
			WS_CLIPCHILDREN |
			WS_CLIPSIBLINGS,
			0, 0,//x and y coords
			windowRect.right - windowRect.left,
			windowRect.bottom - windowRect.top,//width, height
			NULL,//handle to parent
			NULL,//handle to menu
			0,//application instance
			NULL);//no xtra params

	/*      Check if window creation failed (hwnd = null ?)*/
		if (!hwnd)
		{
			return 0;
		}

		ShowWindow(hwnd, SW_SHOW);             //display window
		UpdateWindow(hwnd);//update window

		if (scene3d.isShowFullscreen())
		{
			ChangeDisplaySettings(NULL, 0);
			ShowCursor(TRUE);
		}

		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);

		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LESS);

		//	return int(msg.wParam);
		return 1;
	}

	/**
	 * This loop updates and displays the scene every iteration
	 */
	void Glut::mainLoopWindows()
	{
		while (!m_Glut->getScene3d().isQuit())
		{
			update(0);
			display();
		}
	}
#endif

	/**
	 * http://nehe.gamedev.net/article/replacement_for_gluperspective/21002/
	 * replacement for gluPerspective();
	 */
	void Glut::perspectiveGL(
		GLdouble fovY, GLdouble aspect, GLdouble zNear, GLdouble zFar)
	{
		GLdouble fW, fH;

		fH = tan(fovY / 360 * CV_PI) * zNear;
		fW = fH * aspect;

		glFrustum(-fW, fW, -fH, fH, zNear, zFar);
	}

	void Glut::reset()
	{
		Scene3DRenderer& scene3d = m_Glut->getScene3d();

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		perspectiveGL(50, scene3d.getAspectRatio(), 1, 40000);
		gluLookAt(scene3d.getArcballEye().x, scene3d.getArcballEye().y, scene3d.getArcballEye().z, scene3d.getArcballCentre().x,
			scene3d.getArcballCentre().y, scene3d.getArcballCentre().z, scene3d.getArcballUp().x, scene3d.getArcballUp().y,
			scene3d.getArcballUp().z);

		// set up the ArcBall using the current projection matrix
		arcball_setzoom(scene3d.getSphereRadius(), scene3d.getArcballEye(), scene3d.getArcballUp());

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}

	void Glut::quit()
	{
		m_Glut->getScene3d().setQuit(true);
		exit(EXIT_SUCCESS);
	}

	/**
	 * Handle all keyboard input
	 */
	void Glut::keyboard(
		unsigned char key, int x, int y)
	{
		char* p_end;
		int key_i = strtol(string(key, key).substr(0, 1).c_str(), &p_end, 10);

		Scene3DRenderer& scene3d = m_Glut->getScene3d();
		if (key_i == 0)
		{
			if (key == 'q' || key == 'Q')
			{
				scene3d.setQuit(true);
			}
			else if (key == 'p' || key == 'P')
			{
				bool paused = scene3d.isPaused();
				scene3d.setPaused(!paused);
			}
			else if (key == 'b' || key == 'B')
			{
				scene3d.setCurrentFrame(scene3d.getCurrentFrame() - 1);
			}
			else if (key == 'n' || key == 'N')
			{
				scene3d.setCurrentFrame(scene3d.getCurrentFrame() + 1);
			}
			else if (key == 'r' || key == 'R')
			{
				bool rotate = scene3d.isRotate();
				scene3d.setRotate(!rotate);
			}
			else if (key == 's' || key == 'S')
			{
#ifdef _WIN32
				cerr << "ShowArcball() not supported on Windows!" << endl;
#endif
				bool arcball = scene3d.isShowArcball();
				scene3d.setShowArcball(!arcball);
			}
			else if (key == 'v' || key == 'V')
			{
				bool volume = scene3d.isShowVolume();
				scene3d.setShowVolume(!volume);
			}
			else if (key == 'g' || key == 'G')
			{
				bool floor = scene3d.isShowGrdFlr();
				scene3d.setShowGrdFlr(!floor);
			}
			else if (key == 'c' || key == 'C')
			{
				bool cam = scene3d.isShowCam();
				scene3d.setShowCam(!cam);
			}
			else if (key == 'i' || key == 'I')
			{
#ifdef _WIN32
				cerr << "ShowInfo() not supported on Windows!" << endl;
#endif
				bool info = scene3d.isShowInfo();
				scene3d.setShowInfo(!info);
			}
			else if (key == 'o' || key == 'O')
			{
				bool origin = scene3d.isShowOrg();
				scene3d.setShowOrg(!origin);
			}
			else if (key == 't' || key == 'T')
			{
				scene3d.setTopView();
				reset();
				arcball_reset();
			}
		}
		else if (key_i > 0 && key_i <= (int)scene3d.getCameras().size())
		{
			scene3d.setCamera(key_i - 1);
			reset();
			arcball_reset();
		}
	}

#ifdef __linux__
	/**
	 * Handle linux mouse input (clicks and scrolls)
	 */
	void Glut::mouse(
		int button, int state, int x, int y)
	{
		if (state == GLUT_DOWN)
		{
			int invert_y = (m_Glut->getScene3d().getHeight() - y) - 1;  // OpenGL viewport coordinates are Cartesian
			arcball_start(x, invert_y);
		}

		// scrollwheel support, handcrafted!
		if (state == GLUT_UP)
		{
			if (button == MOUSE_WHEEL_UP && !m_Glut->getScene3d().isCameraView())
			{
				arcball_add_distance(+250);
			}
			else if (button == MOUSE_WHEEL_DOWN && !m_Glut->getScene3d().isCameraView())
			{
				arcball_add_distance(-250);
			}
		}
	}
#elif defined _WIN32
	/**
	 * Function to set the pixel format for the device context
	 */
	void Glut::SetupPixelFormat(HDC hDC)
	{
		/*      Pixel format index
		 */
		int nPixelFormat;

		static PIXELFORMATDESCRIPTOR pfd =
		{
			sizeof(PIXELFORMATDESCRIPTOR),          //size of structure
			1,//default version
			PFD_DRAW_TO_WINDOW |//window drawing support
			PFD_SUPPORT_OPENGL |//opengl support
			PFD_DOUBLEBUFFER,//double buffering support
			PFD_TYPE_RGBA,//RGBA color mode
			32,//32 bit color mode
			0, 0, 0, 0, 0, 0,//ignore color bits
			0,//no alpha buffer
			0,//ignore shift bit
			0,//no accumulation buffer
			0, 0, 0, 0,//ignore accumulation bits
			16,//16 bit z-buffer size
			0,//no stencil buffer
			0,//no aux buffer
			PFD_MAIN_PLANE,//main drawing plane
			0,//reserved
			0, 0, 0 };                              //layer masks ignored

		/*      Choose best matching format*/
		nPixelFormat = ChoosePixelFormat(hDC, &pfd);

		/*      Set the pixel format to the device context*/
		SetPixelFormat(hDC, nPixelFormat, &pfd);
	}

	/**
	 * Handle all windows keyboard and mouse inputs with WM_ events
	 */
	LRESULT CALLBACK Glut::WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		Scene3DRenderer& scene3d = m_Glut->getScene3d();

		// Rendering and Device Context variables are declared here.
		static HGLRC hRC;
		static HDC hDC;
		LONG lRet = 1;

		switch (message)
		{
		case WM_CREATE:                              // Window being created
		{
			hDC = GetDC(hwnd);                              // Get current windows device context
			scene3d.setHDC(hDC);
			SetupPixelFormat(hDC);// Call our pixel format setup function

			// Create rendering context and make it current
			hRC = wglCreateContext(hDC);
			wglMakeCurrent(hDC, hRC);
		}
		break;
		case WM_CLOSE:                              // Window is closing
		{
			hDC = GetDC(hwnd);                              // Get current windows device context
			// Deselect rendering context and delete it
			wglMakeCurrent(hDC, NULL);
			wglDeleteContext(hRC);

			// Send quit message to queue
			PostQuitMessage(0);
		}
		break;
		case WM_SIZE:			//Resize window
		{
			reshape(LOWORD(lParam), HIWORD(lParam));
		}
		break;
		case WM_CHAR:
		{
			keyboard((unsigned char)LOWORD(wParam), 0, 0);
		}
		break;
		case WM_LBUTTONDOWN:			// Left mouse button down
		{
			int x = (int)LOWORD(lParam);
			int y = (int)HIWORD(lParam);
			const int invert_y = (m_Glut->getScene3d().getHeight() - y) - 1;  // OpenGL viewport coordinates are Cartesian
			arcball_start(x, invert_y);
		}
		break;
		case WM_MOUSEMOVE:  // Moving the mouse around
		{
			if (wParam & MK_LBUTTON)  // While left mouse button down
			{
				motion((int)LOWORD(lParam), (int)HIWORD(lParam));
			}
		}
		break;
		case WM_MOUSEWHEEL:  //Scroll wheel
		{
			short zDelta = (short)HIWORD(wParam);
			if (zDelta < 0 && !m_Glut->getScene3d().isCameraView())
			{
				arcball_add_distance(+250);
			}
			else if (zDelta > 0 && !m_Glut->getScene3d().isCameraView())
			{
				arcball_add_distance(-250);
			}
		}
		break;
		default:
			lRet = long(DefWindowProc(hwnd, message, wParam, lParam));
		}

		return lRet;
	}
#endif

	/**
	 * Rotate the scene
	 */
	void Glut::motion(
		int x, int y)
	{
		// motion is only called when a mouse button is held down
		int invert_y = (m_Glut->getScene3d().getHeight() - y) - 1;
		arcball_move(x, invert_y);
	}

	/**
	 * Reshape the GL-window
	 */
	void Glut::reshape(
		int width, int height)
	{
		float ar = (float)width / (float)height;
		m_Glut->getScene3d().setSize(width, height, ar);
		glViewport(0, 0, width, height);
		reset();
	}

	/**
	 * When idle...
	 */
	void Glut::idle()
	{
#ifdef __linux__
		glutPostRedisplay();
#endif
	}

	/**
	 * Render the 3D scene
	 */
	void Glut::display()
	{
		// Enable depth testing
		glEnable(GL_DEPTH_TEST);

		// Here's our rendering. Clears the screen
		// to black, clear the color and depth
		// buffers, and reset our modelview matrix.
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);  //set modelview matrix
		glLoadIdentity();  //reset modelview matrix

		arcball_rotate();

		Scene3DRenderer& scene3d = m_Glut->getScene3d();
		if (scene3d.isShowGrdFlr())
			drawGrdGrid();
		if (scene3d.isShowCam())
			drawCamCoord();
		if (scene3d.isShowVolume())
			drawVolume();
		if (scene3d.isShowArcball())
			drawArcball();

		//canvas = scene3d.getCameras()[scene3d.getPreviousCamera()]->getFrame();


		drawVoxels();

		if (scene3d.isShowOrg())
			drawWCoord();
		if (scene3d.isShowInfo())
			drawInfo();

		glFlush();

#ifdef __linux__
		glutSwapBuffers();
#elif defined _WIN32
		SwapBuffers(scene3d.getHDC());
#endif
	}

	/**
	 * - Update the scene with a new frame from the video
	 * - Handle the keyboard input from the OpenCV window
	 * - Update the OpenCV video window and frames slider position
	 */
	void Glut::update(
		int v)
	{
		char key = waitKey(10);
		keyboard(key, 0, 0);  // call glut key handler :)

		Scene3DRenderer& scene3d = m_Glut->getScene3d();
		if (scene3d.isQuit())
		{
			// Quit signaled
			quit();
		}
		if (scene3d.getCurrentFrame() > scene3d.getNumberOfFrames() - 2)
		{
			// Go to the start of the video if we've moved beyond the end
			scene3d.setCurrentFrame(0);
			for (size_t c = 0; c < scene3d.getCameras().size(); ++c)
				scene3d.getCameras()[c]->setVideoFrame(scene3d.getCurrentFrame());
		}
		if (scene3d.getCurrentFrame() < 0)
		{
			// Go to the end of the video if we've moved before the start
			scene3d.setCurrentFrame(scene3d.getNumberOfFrames() - 2);
			for (size_t c = 0; c < scene3d.getCameras().size(); ++c)
				scene3d.getCameras()[c]->setVideoFrame(scene3d.getCurrentFrame());
		}
		if (!scene3d.isPaused())
		{
			// If not paused move to the next frame
			scene3d.setCurrentFrame(scene3d.getCurrentFrame() + 1);
		}
		if (scene3d.getCurrentFrame() != scene3d.getPreviousFrame())
		{
			// If the current frame is different from the last iteration update stuff
			scene3d.processFrame();
			scene3d.getReconstructor().update();
			scene3d.setPreviousFrame(scene3d.getCurrentFrame());
		}
		else if (scene3d.getHThreshold() != scene3d.getPHThreshold() || scene3d.getSThreshold() != scene3d.getPSThreshold()
			|| scene3d.getVThreshold() != scene3d.getPVThreshold())
		{
			// Update the scene if one of the HSV sliders was moved (when the video is paused)
			scene3d.processFrame();
			scene3d.getReconstructor().update();

			scene3d.setPHThreshold(scene3d.getHThreshold());
			scene3d.setPSThreshold(scene3d.getSThreshold());
			scene3d.setPVThreshold(scene3d.getVThreshold());
		}

		// Auto rotate the scene
		if (scene3d.isRotate())
		{
			arcball_add_angle(2);
		}

		// Get the image and the foreground image (of set camera)
		Mat canvas, foreground;
		if (scene3d.getCurrentCamera() != -1)
		{
			canvas = scene3d.getCameras()[scene3d.getCurrentCamera()]->getFrame();
			foreground = scene3d.getCameras()[scene3d.getCurrentCamera()]->getForegroundImage();
		}
		else
		{
			canvas = scene3d.getCameras()[scene3d.getPreviousCamera()]->getFrame();
			foreground = scene3d.getCameras()[scene3d.getPreviousCamera()]->getForegroundImage();
		}

		// Concatenate the video frame with the foreground image (of set camera)
		if (!canvas.empty() && !foreground.empty())
		{
			Mat fg_im_3c;
			cvtColor(foreground, fg_im_3c, CV_GRAY2BGR);
			hconcat(canvas, fg_im_3c, canvas);
			imshow(VIDEO_WINDOW, canvas);
		}
		else if (!canvas.empty())
		{
			imshow(VIDEO_WINDOW, canvas);
		}

		// Update the frame slider position
		setTrackbarPos("Frame", VIDEO_WINDOW, scene3d.getCurrentFrame());

#ifdef __linux__
		glutSwapBuffers();
		glutTimerFunc(10, update, 0);
#endif
	}

	/**
	 * Draw the floor
	 */
	void Glut::drawGrdGrid()
	{
		vector<vector<Point3i*> > floor_grid = m_Glut->getScene3d().getFloorGrid();

		glLineWidth(1.0f);
		glPushMatrix();
		glBegin(GL_LINES);

		int gSize = m_Glut->getScene3d().getNum() * 2 + 1;
		for (int g = 0; g < gSize; g++)
		{
			// y lines
			glColor4f(0.9f, 0.9f, 0.9f, 0.5f);
			glVertex3f((GLfloat)floor_grid[0][g]->x, (GLfloat)floor_grid[0][g]->y, (GLfloat)floor_grid[0][g]->z);
			glVertex3f((GLfloat)floor_grid[2][g]->x, (GLfloat)floor_grid[2][g]->y, (GLfloat)floor_grid[2][g]->z);

			// x lines
			glColor4f(0.9f, 0.9f, 0.9f, 0.5f);
			glVertex3f((GLfloat)floor_grid[1][g]->x, (GLfloat)floor_grid[1][g]->y, (GLfloat)floor_grid[1][g]->z);
			glVertex3f((GLfloat)floor_grid[3][g]->x, (GLfloat)floor_grid[3][g]->y, (GLfloat)floor_grid[3][g]->z);
		}

		glEnd();
		glPopMatrix();
	}

	/**
	 * Draw the cameras
	 */
	void Glut::drawCamCoord()
	{
		vector<Camera*> cameras = m_Glut->getScene3d().getCameras();

		glLineWidth(1.0f);
		glPushMatrix();
		glBegin(GL_LINES);

		for (size_t i = 0; i < cameras.size(); i++)
		{
			vector<Point3f> plane = cameras[i]->getCameraPlane();

			// 0 - 1
			glColor4f(0.8f, 0.8f, 0.8f, 0.5f);
			glVertex3f(plane[0].x, plane[0].y, plane[0].z);
			glVertex3f(plane[1].x, plane[1].y, plane[1].z);

			// 0 - 2
			glColor4f(0.8f, 0.8f, 0.8f, 0.5f);
			glVertex3f(plane[0].x, plane[0].y, plane[0].z);
			glVertex3f(plane[2].x, plane[2].y, plane[2].z);

			// 0 - 3
			glColor4f(0.8f, 0.8f, 0.8f, 0.5f);
			glVertex3f(plane[0].x, plane[0].y, plane[0].z);
			glVertex3f(plane[3].x, plane[3].y, plane[3].z);

			// 0 - 4
			glColor4f(0.8f, 0.8f, 0.8f, 0.5f);
			glVertex3f(plane[0].x, plane[0].y, plane[0].z);
			glVertex3f(plane[4].x, plane[4].y, plane[4].z);

			// 1 - 2
			glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
			glVertex3f(plane[1].x, plane[1].y, plane[1].z);
			glVertex3f(plane[2].x, plane[2].y, plane[2].z);

			// 2 - 3
			glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
			glVertex3f(plane[2].x, plane[2].y, plane[2].z);
			glVertex3f(plane[3].x, plane[3].y, plane[3].z);

			// 3 - 4
			glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
			glVertex3f(plane[3].x, plane[3].y, plane[3].z);
			glVertex3f(plane[4].x, plane[4].y, plane[4].z);

			// 4 - 1
			glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
			glVertex3f(plane[4].x, plane[4].y, plane[4].z);
			glVertex3f(plane[1].x, plane[1].y, plane[1].z);
		}

		glEnd();
		glPopMatrix();
	}

	/**
	 * Draw the voxel bounding box
	 */
	void Glut::drawVolume()
	{
		vector<Point3f*> corners = m_Glut->getScene3d().getReconstructor().getCorners();

		glLineWidth(1.0f);
		glPushMatrix();
		glBegin(GL_LINES);

		// VR->volumeCorners[0]; // what's this frank?
		// bottom
		glColor4f(0.9f, 0.9f, 0.9f, 0.5f);
		glVertex3f(corners[0]->x, corners[0]->y, corners[0]->z);
		glVertex3f(corners[1]->x, corners[1]->y, corners[1]->z);

		glColor4f(0.9f, 0.9f, 0.9f, 0.5f);
		glVertex3f(corners[1]->x, corners[1]->y, corners[1]->z);
		glVertex3f(corners[2]->x, corners[2]->y, corners[2]->z);

		glColor4f(0.9f, 0.9f, 0.9f, 0.5f);
		glVertex3f(corners[2]->x, corners[2]->y, corners[2]->z);
		glVertex3f(corners[3]->x, corners[3]->y, corners[3]->z);

		glColor4f(0.9f, 0.9f, 0.9f, 0.5f);
		glVertex3f(corners[3]->x, corners[3]->y, corners[3]->z);
		glVertex3f(corners[0]->x, corners[0]->y, corners[0]->z);

		// top
		glColor4f(0.9f, 0.9f, 0.9f, 0.5f);
		glVertex3f(corners[4]->x, corners[4]->y, corners[4]->z);
		glVertex3f(corners[5]->x, corners[5]->y, corners[5]->z);

		glColor4f(0.9f, 0.9f, 0.9f, 0.5f);
		glVertex3f(corners[5]->x, corners[5]->y, corners[5]->z);
		glVertex3f(corners[6]->x, corners[6]->y, corners[6]->z);

		glColor4f(0.9f, 0.9f, 0.9f, 0.5f);
		glVertex3f(corners[6]->x, corners[6]->y, corners[6]->z);
		glVertex3f(corners[7]->x, corners[7]->y, corners[7]->z);

		glColor4f(0.9f, 0.9f, 0.9f, 0.5f);
		glVertex3f(corners[7]->x, corners[7]->y, corners[7]->z);
		glVertex3f(corners[4]->x, corners[4]->y, corners[4]->z);

		// connection
		glColor4f(0.9f, 0.9f, 0.9f, 0.5f);
		glVertex3f(corners[0]->x, corners[0]->y, corners[0]->z);
		glVertex3f(corners[4]->x, corners[4]->y, corners[4]->z);

		glColor4f(0.9f, 0.9f, 0.9f, 0.5f);
		glVertex3f(corners[1]->x, corners[1]->y, corners[1]->z);
		glVertex3f(corners[5]->x, corners[5]->y, corners[5]->z);

		glColor4f(0.9f, 0.9f, 0.9f, 0.5f);
		glVertex3f(corners[2]->x, corners[2]->y, corners[2]->z);
		glVertex3f(corners[6]->x, corners[6]->y, corners[6]->z);

		glColor4f(0.9f, 0.9f, 0.9f, 0.5f);
		glVertex3f(corners[3]->x, corners[3]->y, corners[3]->z);
		glVertex3f(corners[7]->x, corners[7]->y, corners[7]->z);

		glEnd();
		glPopMatrix();
	}

	/**
	 * Draw the arcball wiresphere that guides scene rotation
	 */
	void Glut::drawArcball()
	{
		//Arcball wiresphere (glutWireSphere) not supported on Windows! :(
#ifndef _WIN32
		glLineWidth(1.0f);
		glPushMatrix();
		glBegin(GL_LINES);

		glColor3f(1.0f, 0.9f, 0.9f);
		glutWireSphere(m_Glut->getScene3d().getSphereRadius(), 48, 24);

		glEnd();
		glPopMatrix();
#endif
	}

	float computeMedian(vector<int> elements)
	{
		nth_element(elements.begin(), elements.begin() + elements.size() / 2, elements.end());

		//sort(elements.begin(),elements.end());
		//return float((elements[elements.size() / 2]) / 255);
		return (elements[elements.size() / 2]);
	}

	void makeMat(int k, vector<vector<vector<Vec3b>>>& channels, vector<vector<Mat>>& output_images) {
		// init the proper size
		int max_size = 0;
		for (int c = 0; c < k; c++)
		{
			for (int z = 0; z < k; z++)
			{
				if (channels[c][z].size() > max_size) {
					max_size = channels[c][z].size();
				}
			}
		}


		for (int ca = 0; ca < k; ca++)
		{
			for (int i = 0; i < k; i++) {
				// initlize mat size 
				vector<Vec3b> cur_channel = channels[ca][i];
				output_images[ca][i] = Mat(1, max_size, CV_8UC3, Scalar(0, 0, 0));

				for (int j = 0; j < channels[ca][i].size(); j++) {
					output_images[ca][i].at<Vec3b>(0, j) = cur_channel[j];
				}
			}
		}
	}


	//source: https://www.delftstack.com/pt/howto/cpp/find-most-frequent-element-in-an-array-cpp/
	int getMostFrequentElement(vector<int>& arr)
	{
		if (arr.empty())
			return -1;

		sort(arr.begin(), arr.end());

		auto last_int = arr.front();
		auto most_freq_int = arr.front();
		int max_freq = 0, current_freq = 0;

		for (const auto& i : arr) {
			if (i == last_int)
				++current_freq;
			else {
				if (current_freq > max_freq) {
					max_freq = current_freq;
					most_freq_int = last_int;
				}

				last_int = i;
				current_freq = 1;
			}
		}

		if (current_freq > max_freq) {
			max_freq = current_freq;
			most_freq_int = last_int;
		}

		return most_freq_int;
	}



	void offline_makeMat(int k, vector<vector<Vec3b>> channels, vector<Mat>& output_images) {
		// init the proper size
		int max_size = 0;
		for (int z = 0; z < k; z++)
		{
			if (channels[z].size() > max_size) {
				max_size = channels[z].size();
			}
		}

		for (int i = 0; i < k; i++) {
			// initlize mat size 
			vector<Vec3b> cur_channel = channels[i];
			output_images[i] = Mat(1, max_size, CV_8UC3, Scalar(0, 0, 0));

			for (int j = 0; j < channels[i].size(); j++) {
				output_images[i].at<Vec3b>(0, j) = cur_channel[j];
			}
		}
	}


	void offline_calc_hist(int k, vector<Mat>& ClusteredColorMap, vector<Mat>& Histoutput) {
		int num_bins = 16;
		float xranges[] = { 0, 255 };
		const float* ranges[] = { xranges, xranges, xranges };
		int hist_size[] = { num_bins, num_bins, num_bins };
		int channels[] = { 0,1, 2 };

		for (size_t i = 0; i < k; i++)
		{
			vector<Mat> bgr_planes;
			split(ClusteredColorMap[i], bgr_planes);
			Mat mask(ClusteredColorMap[i].rows, ClusteredColorMap[i].cols, CV_8UC3, Scalar(0, 0, 0));

			// trick gotten from https://stackoverflow.com/questions/36689915/opencv-ignoring-pixels-when-generating-a-hue-histogram
			mask = (bgr_planes[2] == 0) + (bgr_planes[2] == 100) + (bgr_planes[1] == 0);
			bitwise_not(mask, mask);

			calcHist(&ClusteredColorMap[i], 1, channels, mask, Histoutput[i], 3, hist_size, ranges, true, false);
			normalize(Histoutput[i], Histoutput[i], 0, 1, NORM_MINMAX, -1, Mat());

		}
	}


	vector<double> offline_hist_comp(Mat offline_hist, int camera) {
		double base_half_max = 0;
		float min_cor = 0;
		int idx_max = -1;
		int major_lbl;

		vector<double> conf_list(4);

		for (int j = 0; j < HistBase[camera].size(); j++) {
			double base_base = compareHist(offline_hist, HistBase[camera][j], HISTCMP_CORREL);
			conf_list[j] = base_base;
		}
		return conf_list;
	}


	void calculate_hist(int k, bool init, vector<vector<Mat>>& ClusteredColorMap, vector<vector<Mat>>& Histoutput, vector<vector< Reconstructor::Voxel*>>& voxel_cluster, vector<Point2f>& centers) {

		int num_bins = 16;
		float xranges[] = { 0, 255 };
		const float* ranges[] = { xranges, xranges, xranges };
		int hist_size[] = { num_bins, num_bins, num_bins };
		int channels[] = { 0,1, 2 };

		for (int c = 0; c < k; c++)
		{
			for (size_t i = 0; i < k; i++)
			{
				vector<Mat> bgr_planes;
				split(ClusteredColorMap[c][i], bgr_planes);
				Mat mask(ClusteredColorMap[c][i].rows, ClusteredColorMap[c][i].cols, CV_8UC3, Scalar(0, 0, 0));

				// trick gotten from https://stackoverflow.com/questions/36689915/opencv-ignoring-pixels-when-generating-a-hue-histogram
				mask = (bgr_planes[2] == 0) + (bgr_planes[2] == 100) + (bgr_planes[1] == 0);
				bitwise_not(mask, mask);

				calcHist(&ClusteredColorMap[c][i], 1, channels, mask, Histoutput[c][i], 3, hist_size, ranges, true, false);
				normalize(Histoutput[c][i], Histoutput[c][i], 0, 1, NORM_MINMAX, -1, Mat());

				//if (init) {
				//	HistBase[c].push_back(Histoutput[c][i]);
				//}
			}
		}

		//ONLINE
		if (true) {
			// apply default translation
			glTranslatef(0, 0, 0);
			glPointSize(2.0f);
			glPushMatrix();
			glBegin(GL_POINTS);

			vector<int> cameras_classes(4);
			vector<int> cameras_classes_clone(4);
			vector<vector<double>> cameras_classes_conf_values(4, vector<double>(4));


			//loop number of online cameras
			for (int on_cam = 0; on_cam < Histoutput.size(); on_cam++) {
				double base_half_max = 0;
				float min_cor = 0;
				int idx_max = -1;
				int major_lbl;
				vector<int> camera_class_votes;
				vector<double> conf_cam_values(4);


				//loop number of classes 
				for (int on_cl = 0; on_cl < Histoutput[on_cam].size(); on_cl++)
				{
					//same class, different cameras
					Mat cur_online_hist = Histoutput[on_cl][on_cam];
					vector<vector<double>> confidance(4, vector<double>(4));

					//compare the same class, with different offline cameras
					for (int off_cam = 0; off_cam < HistBase.size(); off_cam++)
					{
						confidance[off_cam] = offline_hist_comp(cur_online_hist, off_cam);
					}

					vector<int> conf_order_idx(4);

					vector<double> sum_best_idx_conf(4);

					//frame confidance for each offline camera model
					for (int i = 0; i < confidance.size(); i++)
					{
						double conf = 0;
						double best_conf = 0;
						int best_conf_idx = -1;
						for (int j = 0; j < confidance[i].size(); j++)
						{
							if (confidance[i][j] > best_conf) {
								best_conf = confidance[i][j];
								best_conf_idx = j;
							}
						}
						//camera i votes on best_conf_idx
						conf_order_idx[i] = best_conf_idx;
						//camera i votes with best_conf value
						for (int z = 0; z < confidance.size(); z++)
						{
							for (int zi = 0; zi < confidance[z].size(); zi++)
							{
								sum_best_idx_conf[z] += confidance[zi][z];

							}
						}
					}

					for (int w = 0; w < sum_best_idx_conf.size(); w++)
					{
						conf_cam_values[w] += sum_best_idx_conf[w];

					}


					major_lbl = getMostFrequentElement(conf_order_idx);
					camera_class_votes.push_back(major_lbl);
				}

				//after this, the online cluster class changes
				for (int p = 0; p < conf_cam_values.size(); p++)
				{
					cameras_classes_conf_values[on_cam][p] = conf_cam_values[p];

				}
				cameras_classes[on_cam] = getMostFrequentElement(camera_class_votes);
			}


			cameras_classes_clone = cameras_classes;
			sort(cameras_classes_clone.begin(), cameras_classes_clone.end());
			cameras_classes_clone.erase(unique(cameras_classes_clone.begin(), cameras_classes_clone.end()), cameras_classes_clone.end());

			if (cameras_classes_clone.size() < 4) {
				//back up 
				//cameras_classes.clear(); for some reason reduces the size??
				for (int f = 0; f < cameras_classes.size(); f++)
				{
					cameras_classes[f] = 0;
				}

				//vector<int> new_cameras_classes(4);

				//while (new_cameras_classes.size() != 4) {
				for (int i = 0; i < cameras_classes.size(); i++)
				{
					int best_c = -1;
					int best_r = -1;
					double best_val = 0;
					for (int c = 0; c < cameras_classes_conf_values.size(); c++)
					{
						for (int r = 0; r < cameras_classes_conf_values[c].size(); r++)
						{
							if (cameras_classes_conf_values[c][r] > best_val) {
								best_val = cameras_classes_conf_values[c][r];
								best_r = r;
								best_c = c;
							}
						}

					}
					cameras_classes[best_c] = best_r;
					//clear
					for (int clean_c = 0; clean_c < cameras_classes_conf_values.size(); clean_c++)
					{
						for (int clean_r = 0; clean_r < cameras_classes_conf_values[clean_c].size(); clean_r++)
						{
							cameras_classes_conf_values[best_c][clean_r] = 0;
							cameras_classes_conf_values[clean_r][best_r] = 0;

						}
					}

				}



				//for (int c = 0; c < cameras_classes_conf_values.size(); c++)
				//{
				//	bool complete = false;
				//	while (!complete) {
				//		double conf = 0;
				//		double best_conf = 0;
				//		int best_conf_idx = -1;
				//		for (int r = 0; r < cameras_classes_conf_values[c].size(); r++)
				//		{
				//			if (cameras_classes_conf_values[c][r] > best_conf) {
				//				best_conf = cameras_classes_conf_values[c][r];
				//				best_conf_idx = r;
				//			}
				//		}
				//		//check it has the highest confidence over them all of the same class
				//		if (c < 3) {
				//			bool clear_col = true;
				//			for (int h = 0; h < cameras_classes_conf_values[c].size(); h++)
				//			{
				//				if (cameras_classes_conf_values[h][best_conf_idx] > best_conf) {
				//					best_conf = cameras_classes_conf_values[c][best_conf_idx];
				//					cameras_classes_conf_values[c][best_conf_idx] = 0;
				//					clear_col = false;
				//				}
				//			}

				//			if (clear_col) {
				//				//clean up
				//				for (int d = 0; d < cameras_classes_conf_values[c].size(); d++)
				//				{
				//					cameras_classes_conf_values[d][best_conf_idx] = 0;
				//				}
				//				cameras_classes[c] = best_conf_idx;
				//				complete = true;
				//			}
				//		}
				//		else {
				//			cameras_classes[c] = best_conf_idx;
				//			complete = true;
				//		}
				//		
				//	}
				//}
			}


			for (int lbl = 0; lbl < cameras_classes.size(); lbl++)
			{
				//Point2f last_point;
				//if (!centers_history[cameras_classes[lbl]].empty()) {
				//	last_point = centers_history[cameras_classes[lbl]].back();

				//	if ((abs(centers[lbl].x - last_point.x) < 20) || (abs(centers[lbl].y - last_point.y) < 20))
				//		centers_history[cameras_classes[lbl]].push_back(centers[lbl]);
				//}
				//else
				centers_history[cameras_classes[lbl]].push_back(centers[lbl]);



				for (int z = 0; z < voxel_cluster[lbl].size(); z++)
				{
					voxel_cluster[lbl][z]->label = cameras_classes[lbl];
					float R_, G_, B_;
					switch (voxel_cluster[lbl][z]->label) {
					case 0:
						R_ = 0.8;
						G_ = 0.0;
						B_ = 0.0;
						break;
					case 1:
						R_ = 0.0;
						G_ = 0.8;
						B_ = 0.0;
						break;
					case 2:
						R_ = 0.0;
						G_ = 0.0;
						B_ = 0.8;
						break;
					case 3:
						R_ = 0.8;
						G_ = 0.0;
						B_ = 0.8;
						break;
					default:
						R_ = 0.5;
						G_ = 0.5;
						B_ = 0.5;
						break;
					}

					glColor4f(R_, G_, B_, 0.5f);
					voxel_cluster[lbl][z]->color = Scalar(R_, G_, B_, 0.5f);
					glVertex3f((GLfloat)voxel_cluster[lbl][z]->x, (GLfloat)voxel_cluster[lbl][z]->y, (GLfloat)voxel_cluster[lbl][z]->z);
				}
			}
			glEnd();
			glPopMatrix();
		}
	}

	//void Glut::checkOcclusion(std::vector<cv::Mat> ids, std::vector<cv::Mat> dis) {

	//};


	//code from https://docs.opencv.org/3.4/d8/dbc/tutorial_histogram_calculation.html
	//void build_histogram(Mat image3, Mat& histImage) {
	//	vector<Mat> bgr_planes;
	//	Mat img_hsv;
	//	cvtColor(image3, img_hsv, COLOR_BGR2HSV);
	//	split(img_hsv, bgr_planes);
	//
	//	int histSize = 256;
	//	float range[] = { 0,256 };
	//	const float* histRange = { range };
	//	bool uniform = true, accumulate = false;
	//
	//	Mat b_hist, g_hist, r_hist, img_and, img_not;
	//	Mat mask(image3.rows, image3.cols, CV_8UC3, Scalar(0, 0, 0));
	//
	//	// trick gotten from https://stackoverflow.com/questions/36689915/opencv-ignoring-pixels-when-generating-a-hue-histogram
	//	mask = (bgr_planes[2] == 0) + (bgr_planes[2] == 100) + (bgr_planes[1] == 0);
	//	bitwise_not(mask, mask);
	//
	//	calcHist(&bgr_planes[0], 1, 0, mask, b_hist, 1, &histSize, &histRange, uniform, accumulate);
	//	calcHist(&bgr_planes[1], 1, 0, mask, g_hist, 1, &histSize, &histRange, uniform, accumulate);
	//	calcHist(&bgr_planes[2], 1, 0, mask, r_hist, 1, &histSize, &histRange, uniform, accumulate);
	//	
	//	int hist_w = 512, hist_h = 400;
	//	int bin_w = cvRound((double)hist_w / histSize);
	//
	//
	//	normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
	//	normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
	//	normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
	//
	//	for (int i = 1; i < histSize; i++)
	//	{
	//		line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
	//			Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))),
	//			Scalar(255, 0, 0), 2, 8, 0);
	//		line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
	//			Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))),
	//			Scalar(0, 255, 0), 2, 8, 0);
	//		line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
	//			Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))),
	//			Scalar(0, 0, 255), 2, 8, 0);
	//	}
	//}

	vector<Point2f> interpolation(vector<Point2f> centers, int inter) {
		vector<Point2f> output;
		for (int i = 0; i < centers.size() - inter; i++) {
			Point2f cur = Point2f(0, 0);
			for (int k = i; k < i + inter; k++) {
				cur += centers[k];}
			output.push_back(cur / inter);
		}
		return output;
	}

	void paint_path() {
		glPushMatrix();


		for (int c = 0; c < centers_history.size(); c++)
		{
			vector<Point2f> interpolate_center = interpolation(centers_history[c], 5);

			for (int i = 9; i < interpolate_center.size()-3; i++)
			{
				float R_, G_, B_;
				switch (c) {
				case 0:
					R_ = 0.8;
					G_ = 0.0;
					B_ = 0.0;
					break;
				case 1:
					R_ = 0.0;
					G_ = 0.8;
					B_ = 0.0;
					break;
				case 2:
					R_ = 0.0;
					G_ = 0.0;
					B_ = 0.8;
					break;
				case 3:
					R_ = 0.8;
					G_ = 0.0;
					B_ = 0.8;
					break;
				default:
					R_ = 0.5;
					G_ = 0.5;
					B_ = 0.5;
					break;
				}
				glColor4f(R_, G_, B_, 0.5f);

				glLineWidth(1.0f);
				glBegin(GL_LINES);
				glVertex3f((GLfloat)interpolate_center[i].x, (GLfloat)interpolate_center[i].y, 0);
				if ((i + 1) < centers_history[c].size() - 1)
					glVertex3f((GLfloat)interpolate_center[(i + 1)].x, (GLfloat)interpolate_center[(i + 1)].y, 0);
			}
		}
		glEnd();
		glPopMatrix();
	}



	/**
	 * Draw all visible voxels
	 */
	void Glut::drawVoxels()
	{
		glPushMatrix();

		// apply default translation
		glTranslatef(0, 0, 0);
		glPointSize(2.0f);
		glBegin(GL_POINTS);

		//if (!initialized_coloring) {
		Scene3DRenderer& scene3d = m_Glut->getScene3d();
		Mat canvas;
		vector<int> elements_B;
		vector<int> elements_G;
		vector<int> elements_R;
		vector<int> voxel_labels;
		int K = 4;
		vector<int> camera_order = { 1,0,3,2 };
		Mat data;

		vector<Reconstructor::Voxel*> voxels = m_Glut->getScene3d().getReconstructor().getVisibleVoxels();

		vector<Mat> VoxelId = m_Glut->getScene3d().getReconstructor().getShortestVoxelId();
		vector<Mat> VoxelDis = m_Glut->getScene3d().getReconstructor().getShortestVoxelDistance();

		vector<Reconstructor::Voxel*> not_occluded_voxels = vector<Reconstructor::Voxel*>(1);


		/*ONLINE*/
		if (initialized_coloring) {
			vector<Point2f> m_groundCoordinates(voxels.size());

			for (int v = 0; v < (int)voxels.size(); v++) {
				m_groundCoordinates[v] = Point2f(voxels[v]->x, voxels[v]->y);
				//if (checkOcclusion(VoxelId, VoxelDis, voxels[v]->id, voxels[v]->distanceCamera, voxels[v]->cornes_vamera_projection)) {
				//	not_occluded_voxels.push_back(voxels[v]);
				//};
			}
			//bool checkOcclusion(std::vector<Mat> ids, std::vector<Mat> distances, int id, std::vector<float> disVoxel, std::vector<Point> voxelProjections) {

			vector<int> labels;
			vector<Point2f> centers;

			kmeans(m_groundCoordinates, K, labels, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1.0), 5, KMEANS_PP_CENTERS, centers);

			vector<int> lbl = labels;
			vector<Point2f> cen = centers;

			sort(lbl.begin(), lbl.end());
			lbl.erase(unique(lbl.begin(), lbl.end()), lbl.end());
			cen.erase(unique(cen.begin(), cen.end()), cen.end());


			if (lbl.size() < 4 || cen.size() < 4) {
				cout << "Labels " << lbl.size() << "\n";
				cout << "centers " << cen.size() << "\n";
			}

			//camera - cluster class - voxel coords
			vector<vector<vector<Vec3b>>> colors(4, vector< vector<Vec3b> >(4));
			vector<vector< Reconstructor::Voxel*>> cluster_voxels(4);


			//Projecting voxels to the selected camera image 2
			for (int c_idx = 0; c_idx < scene3d.getCameras().size(); c_idx++)
			{
				int c = camera_order[c_idx];
				canvas = scene3d.getCameras()[c]->getFrame();

				for (int v = 0; v < voxels.size(); v++) {
					if ((voxels[v]->z > 600 || voxels[v]->z < 200) && voxels[v]->valid_camera_projection[c]) {
						colors[c][labels[v]].push_back(canvas.at<Vec3b>(voxels[v]->camera_projection[c]));
					}


					if (c == 0) {
						cluster_voxels[labels[v]].push_back(voxels[v]);
					}
				}
			}


			vector<vector<Mat>> temp_classes(K, vector<Mat>(K));
			makeMat(K, colors, temp_classes);
			vector<vector<MatND>> hist_base(4, vector<MatND>(4));
			calculate_hist(K, false, temp_classes, hist_base, cluster_voxels, centers);
			if (centers_history[1].size() > 10)
				paint_path();


			/*OFFLINE*/
		}
		else if (voxels.size() > 0 && !initialized_coloring) {
			initialized_coloring = true;


			//camera - cluster class - voxel coords v(n, vector< vector<int> >(m , vector<int>(l)));
			vector<vector<vector<Vec3b>>> colors(4, vector< vector<Vec3b> >(4));

			vector<vector< Reconstructor::Voxel*>> cluster_voxels(K);

			vector<vector<double>> confidance(4, vector<double>(4));


			vector<int> best_camera_frames = { 666, 2, 644, 902 };

			//Projecting voxels to the selected camera image 2
			for (size_t c_idx = 0; c_idx < scene3d.getCameras().size(); c_idx++)
			{

				int c = camera_order[c_idx];

				scene3d.setCurrentFrame(best_camera_frames[c]);
				scene3d.getCameras()[c]->setVideoFrame(best_camera_frames[c]);

				scene3d.processFrame();
				scene3d.getReconstructor().update();
				canvas = scene3d.getCameras()[c]->getFrame();
				voxels = m_Glut->getScene3d().getReconstructor().getVisibleVoxels();

				vector<Point2f> m_groundCoordinates(voxels.size());
				vector<vector<Vec3b>> colors_offline(4);
				vector<int> initialized_labels;
				vector<Point2f> initialized_centers;



				for (int v = 0; v < (int)voxels.size(); v++) {
					m_groundCoordinates[v] = Point2f(voxels[v]->x, voxels[v]->y);
				}
				kmeans(m_groundCoordinates, K, initialized_labels, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1.0), 5, KMEANS_PP_CENTERS, initialized_centers);


				for (int v = 0; v < voxels.size(); v++) {
					if ((voxels[v]->z > 600 || voxels[v]->z < 200) && voxels[v]->valid_camera_projection[c]) {
						colors_offline[initialized_labels[v]].push_back(canvas.at<Vec3b>(voxels[v]->camera_projection[c]));
					}
				}



				vector<Mat> temp_images(K);
				offline_makeMat(K, colors_offline, temp_images);
				vector<MatND> hist_base(4);
				offline_calc_hist(K, temp_images, hist_base);


				if (!HistBase[0].empty()) {
					for (int off = 0; off < hist_base.size(); off++)
					{
						confidance[off] = offline_hist_comp(hist_base[off], 0);
					}
					vector<int> conf_order_idx(4);
					for (int i = 0; i < confidance.size(); i++)
					{
						double conf = 0;
						double best_conf = 0;
						int best_conf_idx = -1;
						for (int j = 0; j < confidance[i].size(); j++)
						{
							if (confidance[j][i] > best_conf) {
								best_conf = confidance[j][i];
								best_conf_idx = j;
							}
						}
						conf_order_idx[i] = best_conf_idx;
						for (int z = 0; z < confidance[i].size(); z++)
						{
							confidance[z][i] = 0;
						}
					}

					for (int y = 0; y < hist_base.size(); y++)
					{
						HistBase[c_idx].push_back(hist_base[conf_order_idx[y]]);
					}

				}
				else {
					for (int h = 0; h < hist_base.size(); h++)
					{
						HistBase[0].push_back(hist_base[h]);
					}
				}

			}
			scene3d.setCurrentFrame(0);
			for (size_t c = 0; c < scene3d.getCameras().size(); ++c)
				scene3d.getCameras()[c]->setVideoFrame(scene3d.getCurrentFrame());
			scene3d.processFrame();
			scene3d.getReconstructor().update();
		}

		glEnd();
		glPopMatrix();
	}

	/**
	 * Draw origin into scene
	 */
	void Glut::drawWCoord()
	{
		glLineWidth(1.5f);
		glPushMatrix();
		glBegin(GL_LINES);

		const Scene3DRenderer& scene3d = m_Glut->getScene3d();
		const int len = scene3d.getSquareSideLen();
		const float x_len = float(len * (scene3d.getBoardSize().height - 1));
		const float y_len = float(len * (scene3d.getBoardSize().width - 1));
		const float z_len = float(len * 3);

		// draw x-axis
		glColor4f(0.0f, 0.0f, 1.0f, 0.5f);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(x_len, 0.0f, 0.0f);

		// draw y-axis
		glColor4f(0.0f, 1.0f, 0.0f, 0.5f);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(0.0f, y_len, 0.0f);

		// draw z-axis
		glColor4f(1.0f, 0.0f, 0.0f, 0.5f);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(0.0f, 0.0f, z_len);

		glEnd();
		glPopMatrix();
	}



	/**
	 * Draw camera numbers into scene
	 */
	void Glut::drawInfo()
	{
		// glutBitmapCharacter() is not supported on Windows
#ifndef _WIN32
		glPushMatrix();
		glBegin(GL_BITMAP);

		if (m_Glut->getScene3d().isShowInfo())
		{
			vector<Camera*> cameras = m_Glut->getScene3d().getCameras();
			for (size_t c = 0; c < cameras.size(); ++c)
			{
				glRasterPos3d(cameras[c]->getCameraLocation().x, cameras[c]->getCameraLocation().y, cameras[c]->getCameraLocation().z);
				stringstream sstext;
				sstext << (c + 1) << "\0";
				for (const char* c = sstext.str().c_str(); *c != '\0'; c++)
				{
					glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
				}
			}
		}

		glEnd();
		glPopMatrix();
#endif
	}

} /* namespace nl_uu_science_gmt */