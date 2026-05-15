#pragma once
#include "Vortrix/Core/NonCopyable.h"

struct GLFWwindow;
namespace InputSystem
{
	constexpr int NUM_ACTIONS = 2; //PRESS -- RELEASE 
	constexpr int NUM_KEYS = 1024;

	enum class EInputMode : int
	{
		Release = 0,
		Press = 1
	};

	class EventHandler : public vx::NonCopyable
	{
	public:
		static inline EventHandler& Instance()
		{
			static EventHandler inst;
			return inst;
		}
		void CreateCallbacks(GLFWwindow* window);

		void FlushFrameInputs();

	private:
		EventHandler() {}
		bool mKeys[NUM_KEYS][NUM_ACTIONS] = {false};
		float mMouseDt[2] = {0.0f};
		bool mMouseMoved = false;
		float mScroll[2] = { 0.0f };

		//utility key repeating key over frame
		bool mHeldKey[NUM_KEYS / 2] = { false };

		static void KeyboardInputCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
		static void MouseCursorInputCallback(GLFWwindow* window, double x_pos, double y_pos);
		static void MouseButtonInputCallback(GLFWwindow* window, int button, int action, int mods);
		static void MouseScrollInputCallback(GLFWwindow* window, double x_offset, double y_offset);

		friend class Input;
	};

	enum class IAxis;
	enum class IKeyCode;
	class Input
	{
	public:
		/// <summary>
		/// Returns true during the frame user presses the key identified by IKeyCode
		/// </summary>
		/// <param name="key"></param>
		/// <returns></returns>
		static bool GetKeyDown(IKeyCode key);
		/// <summary>
		/// Returns true during the frame user releases the key identified by IKeyCode
		/// </summary>
		/// <param name="key"></param>
		/// <returns></returns>
		static bool GetKeyUp(IKeyCode key);
		/// <summary>
		/// Returns true while the user hold the key identified by IKeyCode
		/// </summary>
		/// <param name="key"></param>
		/// <returns></returns>
		static bool GetKey(IKeyCode key);
		static bool GetMouse(IKeyCode key);
		static bool GetMousePressed(IKeyCode key);
		static float GetMouseAxisFloat(IAxis axis);

		static float GetScrollWheel();
	};


	enum class IAxis : int
	{
		Vertical = 0,
		Horizontal = 1,
	};// IAxis enum class

	
	//replica of GLFW input
	enum class IKeyCode : int
	{
		Space            =   32,
		Apostrophe       =   39,  /* ' */
		Comma            =   44,  /* , */
		Minus            =   45,  /* - */
		Period           =   46,  /* . */
		Slash            =   47,  /* / */
		Keypad0          =   48,
		Keypad1          =   49,
		Keypad2          =   50,
		Keypad3          =   51,
		Keypad4          =   52,
		Keypad5          =   53,
		Keypad6          =   54,
		Keypad7          =   55,
		Keypad8          =   56,
		Keypad9          =   57,
		Semicolon        =   59,  /* ; */
		Equal            =   61,  /* = */
		A                =   65,
		B                =   66,
		C                =   67,
		D                =   68,
		E                =   69,
		F                =   70,
		G                =   71,
		H                =   72,
		I                =   73,
		J                =   74,
		K                =   75,
		L                =   76,
		M                =   77,
		N                =   78,
		O                =   79,
		P                =   80,
		Q                =   81,
		R                =   82,
		S                =   83,
		T                =   84,
		U                =   85,
		V                =   86,
		W                =   87,
		X                =   88,
		Y                =   89,
		Z                =   90,
		LeftBracket      =   91,  /* [ */
		Backslash        =   92,  /* \ */
		RightBracket     =   93,  /* ] */
		GraveAccent      =   96,  /* ` */
					     
		//Mouse		     
		Mouse0		     =    0,
		Mouse1		     =    1,
		Mouse2		     =    2,
		Mouse3		     =    3,
		Mouse4		     =    4,
		Mouse5		     =    5,
		Mouse6		     =    6,
		MouseLeftButton  =    Mouse0,	
		MouseRightButton =    Mouse1,	
		
		//function
		Escape           =   256,
		Enter            =   257,
		Tab              =   258,
		Backspace        =   259,
		Insert           =   260,
		Delete           =   261,
		RightArrow       =   262,
		LeftArrow        =   263,
		DownArrow        =   264,
		UpArrow          =   265,
		PageUp           =   266,
		PageDown         =   267,
		Home             =   268,
		End              =   269,
		CapsLock         =   280,
		ScrollLock       =   281,
		NumLock          =   282,
		PrintScreen      =   283,
		Pause            =   284,
		F1               =   290,
		F2               =   291,
		F3               =   292,
		F4               =   293,
		F5               =   294,
		F6               =   295,
		F7               =   296,
		F8               =   297,
		F9               =   298,
		F10              =   299,
		F11              =   300,
		F12              =   301,
		F13              =   302,
		F14              =   303,
		F15              =   304,
		F16              =   305,
		F17              =   306,
		F18              =   307,
		F19              =   308,
		F20              =   309,
		F21              =   310,
		F22              =   311,
		F23              =   312,
		F24              =   313,
		F25              =   314,
		KP_0             =   320,
		KP_1             =   321,
		KP_2             =   322,
		KP_3             =   323,
		KP_4             =   324,
		KP_5             =   325,
		KP_6             =   326,
		KP_7             =   327,
		KP_8             =   328,
		KP_9             =   329,
		KP_DECIMAL       =   330,
		KP_DIVIDE        =   331,
		KP_MULTIPLY      =   332,
		KP_SUBTRACT      =   333,
		KP_ADD           =   334,
		KP_ENTER         =   335,
		KP_EQUAL         =   336,
		LeftShift        =   340,
		LeftControl      =   341,
		LeftAlt          =   342,
		LeftSuper        =   343,
		RightShift       =   344,
		RightControl     =   345,
		RightAlt         =   346,
		RightSuper       =   347,
		Menu             =   348,

	};// IKeyCode enum class

}//# InputSystem namespace

