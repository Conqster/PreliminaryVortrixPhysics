#pragma once
#include "Vortrix/Core/NonCopyable.h"

#include <array>
#include <string_view>

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
#define KEY_CODES(X) \
		X(Space, 32)		\
		X(Apostrophe,39)	\
		X(Comma,44)			\
		X(Minus,45)			\
		X(Period ,46)			\
		X(Slash ,47)			\
		X(Keypad0, 48)			\
		X(Keypad1, 49)			\
		X(Keypad2, 50)			\
		X(Keypad3, 51)			\
		X(Keypad4, 52)			\
		X(Keypad5, 53)			\
		X(Keypad6, 54)			\
		X(Keypad7, 55)			\
		X(Keypad8, 56)			\
		X(Keypad9, 57)			\
		X(Semicolon, 59)			\
		X(Equal, 61)			\
		X(A, 65)			\
		X(B, 66)			\
		X(C, 67)			\
		X(D, 68)			\
		X(E, 69)			\
		X(F, 70)			\
		X(G, 71)			\
		X(H, 72)			\
		X(I, 73)			\
		X(J, 74)			\
		X(K, 75)			\
		X(L, 76)			\
		X(M, 77)			\
		X(N, 78)			\
		X(O, 79)			\
		X(P, 80)			\
		X(Q, 81)			\
		X(R, 82)			\
		X(S, 83)			\
		X(T, 84)			\
		X(U, 85)			\
		X(V, 86)			\
		X(W, 87)			\
		X(X, 88)			\
		X(Y, 89)			\
		X(Z, 90)			\
		X(LeftBracket, 91)			\
		X(Backslash, 92)			\
		X(RightBracket, 93)			\
		X(GraveAccent, 96)			\
		X(Mouse0, 0)			\
		X(Mouse1, 1)			\
		X(Mouse2, 2)			\
		X(Mouse3, 3)			\
		X(Mouse4, 4)			\
		X(Mouse5, 5)			\
		X(Mouse6, 6)			\
		X(Escape, 256)		\
		X(Enter, 257)		\
		X(Tab, 258)		\
		X(Backspace, 259)		\
		X(Insert, 260)		\
		X(Delete, 261)		\
		X(RightArrow, 262)		\
		X(LeftArrow, 263)		\
		X(DownArrow, 264)		\
		X(UpArrow, 265)		\
		X(PageUp, 266)		\
		X(PageDown, 267)		\
		X(Home, 268)		\
		X(End, 269)		\
		X(CapsLock, 280)		\
		X(ScrollLock, 281)		\
		X(NumLock, 282)		\
		X(PrintScreen, 283)		\
		X(Pause, 284)		\
		X(F1, 290)		\
		X(F2, 291)		\
		X(F3, 292)		\
		X(F4, 293)		\
		X(F5, 294)		\
		X(F6, 295)		\
		X(F7, 296)		\
		X(F8, 297)		\
		X(F9, 298)		\
		X(F10, 299)		\
		X(F11, 300)		\
		X(F12, 301)		\
		X(F13, 302)		\
		X(F14, 303)		\
		X(F15, 304)		\
		X(F16, 305)		\
		X(F17, 306)		\
		X(F18, 307)		\
		X(F19, 308)		\
		X(F20, 309)		\
		X(F21, 310)		\
		X(F22, 311)		\
		X(F23, 312)		\
		X(F24, 313)		\
		X(F25, 314)		\
		X(KP_0, 320)		\
		X(KP_1, 321)		\
		X(KP_2, 322)		\
		X(KP_3, 323)		\
		X(KP_4, 324)		\
		X(KP_5, 325)		\
		X(KP_6, 326)		\
		X(KP_7, 327)		\
		X(KP_8, 328)		\
		X(KP_9, 329)		\
		X(KP_DECIMAL, 330)		\
		X(KP_DIVIDE, 331)		\
		X(KP_MULTIPLY, 332)		\
		X(KP_SUBTRACT, 333)		\
		X(KP_ADD, 334)		\
		X(KP_ENTER, 335)		\
		X(KP_EQUAL, 336)		\
		X(LeftShift, 340)		\
		X(LeftControl, 341)		\
		X(LeftAlt, 342)		\
		X(LeftSuper, 343)		\
		X(RightShift, 344)		\
		X(RightControl, 345)		\
		X(RightAlt, 346)		\
		X(RightSuper, 347)		\
		X(Menu, 348)		

enum class IKeyCode : int
{
#define X(name, code) name = code,
	KEY_CODES(X)
#undef X


	MouseLeftButton = Mouse0,
	//MouseRightButton = int(Mouse1),
	MouseRightButton = Mouse1
};

//static_assert();

enum class IKeyIndex : uint16_t//vx::uint16
{
#define X(name, code) name,
	KEY_CODES(X)
#undef X

	Count
};

constexpr std::array<std::string_view, static_cast<size_t>(IKeyIndex::Count)> IKeyCodeNames =
{
#define X(name, code) #name,
	KEY_CODES(X)
#undef X
};

constexpr std::array<const char*, static_cast<size_t>(IKeyIndex::Count)> IKeyCodeNamesChar =
{
#define X(name, code) #name,
	KEY_CODES(X)
#undef X
};


namespace internals{
	constexpr std::array<IKeyCode, static_cast<size_t>(IKeyIndex::Count)> IKeyIndexToCodeMap =
	{
	#define X(name, code) IKeyCode::name,
		KEY_CODES(X)
	#undef X
	};

	static constexpr uint16_t kInvalidKeyCodeSlot = uint16_t(-1);
	constexpr auto IKeyCodeToIndexMap = []()
	{
		std::array<uint16_t, 349> temp{};
		//temp.fill(kInvalidKeyCodeSlot);

		for (auto& v : temp)
			v = kInvalidKeyCodeSlot;

#define X(name, code) temp[code] = static_cast<uint16_t>(IKeyIndex::name);
		KEY_CODES(X)
#undef X
		return temp;
	}();
}


constexpr IKeyCode KeyIndexToCode(IKeyIndex index) { return internals::IKeyIndexToCodeMap[static_cast<size_t>(index)];}


constexpr int KeyCodeToIndex(IKeyCode code)
{
	const int v = static_cast<int>(code);
	if (v < 0 || v >= static_cast<int>(internals::IKeyCodeToIndexMap.size()))
		return -1;

	return internals::IKeyCodeToIndexMap[v];
}
//
//
////replica of GLFW input
//enum class IKeyCode : int
//{
//	Space = 32,
//	Apostrophe = 39,  /* ' */
//	Comma = 44,  /* , */
//	Minus = 45,  /* - */
//	Period = 46,  /* . */
//	Slash = 47,  /* / */
//	Keypad0 = 48,
//	Keypad1 = 49,
//	Keypad2 = 50,
//	Keypad3 = 51,
//	Keypad4 = 52,
//	Keypad5 = 53,
//	Keypad6 = 54,
//	Keypad7 = 55,
//	Keypad8 = 56,
//	Keypad9 = 57,
//	Semicolon = 59,  /* ; */
//	Equal = 61,  /* = */
//	A = 65,
//	B = 66,
//	C = 67,
//	D = 68,
//	E = 69,
//	F = 70,
//	G = 71,
//	H = 72,
//	I = 73,
//	J = 74,
//	K = 75,
//	L = 76,
//	M = 77,
//	N = 78,
//	O = 79,
//	P = 80,
//	Q = 81,
//	R = 82,
//	S = 83,
//	T = 84,
//	U = 85,
//	V = 86,
//	W = 87,
//	X = 88,
//	Y = 89,
//	Z = 90,
//	LeftBracket = 91,  /* [ */
//	Backslash = 92,  /* \ */
//	RightBracket = 93,  /* ] */
//	GraveAccent = 96,  /* ` */
//
//	//Mouse		     
//	Mouse0 = 0,
//	Mouse1 = 1,
//	Mouse2 = 2,
//	Mouse3 = 3,
//	Mouse4 = 4,
//	Mouse5 = 5,
//	Mouse6 = 6,
//	MouseLeftButton = Mouse0,
//	MouseRightButton = Mouse1,
//
//	//function
//	Escape = 256,
//	Enter = 257,
//	Tab = 258,
//	Backspace = 259,
//	Insert = 260,
//	Delete = 261,
//	RightArrow = 262,
//	LeftArrow = 263,
//	DownArrow = 264,
//	UpArrow = 265,
//	PageUp = 266,
//	PageDown = 267,
//	Home = 268,
//	End = 269,
//	CapsLock = 280,
//	ScrollLock = 281,
//	NumLock = 282,
//	PrintScreen = 283,
//	Pause = 284,
//	F1 = 290,
//	F2 = 291,
//	F3 = 292,
//	F4 = 293,
//	F5 = 294,
//	F6 = 295,
//	F7 = 296,
//	F8 = 297,
//	F9 = 298,
//	F10 = 299,
//	F11 = 300,
//	F12 = 301,
//	F13 = 302,
//	F14 = 303,
//	F15 = 304,
//	F16 = 305,
//	F17 = 306,
//	F18 = 307,
//	F19 = 308,
//	F20 = 309,
//	F21 = 310,
//	F22 = 311,
//	F23 = 312,
//	F24 = 313,
//	F25 = 314,
//	KP_0 = 320,
//	KP_1 = 321,
//	KP_2 = 322,
//	KP_3 = 323,
//	KP_4 = 324,
//	KP_5 = 325,
//	KP_6 = 326,
//	KP_7 = 327,
//	KP_8 = 328,
//	KP_9 = 329,
//	KP_DECIMAL = 330,
//	KP_DIVIDE = 331,
//	KP_MULTIPLY = 332,
//	KP_SUBTRACT = 333,
//	KP_ADD = 334,
//	KP_ENTER = 335,
//	KP_EQUAL = 336,
//	LeftShift = 340,
//	LeftControl = 341,
//	LeftAlt = 342,
//	LeftSuper = 343,
//	RightShift = 344,
//	RightControl = 345,
//	RightAlt = 346,
//	RightSuper = 347,
//	Menu = 348,
//
//};// IKeyCode enum class
//
//
//
//
//	enum class Code_To_idx : int
//	{
//		Space = 0,
//		Apostrophe,
//		Comma,
//		Minus,
//		Period,
//		Slash,
//		Keypad0,
//		Keypad1,
//		Keypad2,
//		Keypad3,
//		Keypad4,
//		Keypad5,
//		Keypad6,
//		Keypad7,
//		Keypad8,
//		Keypad9,
//		Semicolon,
//		Equal,
//		A,
//		B,
//		C,
//		D,
//		E,
//		F,
//		G,
//		H,
//		I,
//		J,
//		K,
//		L,
//		M,
//		N,
//		O,
//		P,
//		Q,
//		R,
//		S,
//		T,
//		U,
//		V,
//		W,
//		X,
//		Y,
//		Z,
//		LeftBracket,
//		Backslash,
//		RightBracket,
//		GraveAccent,
//
//		//Mouse		     
//		Mouse0,
//		Mouse1,
//		Mouse2,
//		Mouse3,
//		Mouse4,
//		Mouse5,
//		Mouse6,
//		MouseLeftButton,
//		MouseRightButton,
//
//		//function
//		Escape,
//		Enter,
//		Tab,
//		Backspace,
//		Insert,
//		Delete,
//		RightArrow,
//		LeftArrow,
//		DownArrow,
//		UpArrow,
//		PageUp,
//		PageDown,
//		Home,
//		End,
//		CapsLock,
//		ScrollLock,
//		NumLock,
//		PrintScreen,
//		Pause,
//		F1,
//		F2,
//		F3,
//		F4,
//		F5,
//		F6,
//		F7,
//		F8,
//		F9,
//		F10,
//		F11,
//		F12,
//		F13,
//		F14,
//		F15,
//		F16,
//		F17,
//		F18,
//		F19,
//		F20,
//		F21,
//		F22,
//		F23,
//		F24,
//		F25,
//		KP_0 ,
//		KP_1,
//		KP_2,
//		KP_3,
//		KP_4,
//		KP_5,
//		KP_6,
//		KP_7,
//		KP_8,
//		KP_9,
//		KP_DECIMAL,
//		KP_DIVIDE,
//		KP_MULTIPLY,
//		KP_SUBTRACT,
//		KP_ADD,
//		KP_ENTER,
//		KP_EQUAL,
//		LeftShift,
//		LeftControl,
//		LeftAlt,
//		LeftSuper,
//		RightShift,
//		RightControl,
//		RightAlt,
//		RightSuper,
//		Menu,
//
//	};// IKeyCode enum class
//
//
//
//
//	//replica of GLFW input
//	constexpr const char* IKeyCodeNames = 
//		"Space\0"
//		"Apostrophe[']\0"
//		"Comma[,]\0"
//		"Minus[-]\0"
//		"Period[.]\0"
//		"Slash[/]\0"
//		"Keypad0\0"
//		"Keypad1\0"
//		"Keypad2\0"
//		"Keypad3\0"
//		"Keypad4\0"
//		"Keypad5\0"
//		"Keypad6\0"
//		"Keypad7\0"
//		"Keypad8\0"
//		"Keypad9\0"
//		"Semicolon[;]\0"
//		"Equal[=]\0"
//		"A\0"
//		"B\0"
//		"C\0"
//		"D\0"
//		"E\0"
//		"F\0"
//		"G\0"
//		"H\0"
//		"I\0"
//		"J\0"
//		"K\0"
//		"L\0"
//		"M\0"
//		"N\0"
//		"O\0"
//		"P\0"
//		"Q\0"
//		"R\0"
//		"S\0"
//		"T\0"
//		"U\0"
//		"V\0"
//		"W\0"
//		"X\0"
//		"Y\0"
//		"Z\0"
//		"LeftBracket[ [ ]\0"
//		"Backslash[\]\0"
//		"RightBracket [ ] ]\0"
//		"GraveAccent[`]"
//
//		//Mouse		 
//		"Mouse0\0"
//		"Mouse1\0"
//		"Mouse2\0"
//		"Mouse3\0"
//		"Mouse4\0"
//		"Mouse5\0"
//		"Mouse6\0"
//		"MouseLeftButton\0"
//		"MouseRightButton\0"
//
//		//function
//		"Escape\0"
//		"Enter\0"
//		"Tab\0"
//		"Backspace\0"
//		"Insert\0"
//		"Delete\0"
//		"RightArrow\0"
//		"LeftArrow\0"
//		"DownArrow\0"
//		"UpArrow\0"
//		"PageUp\0"
//		"PageDown\0"
//		"Home\0"
//		"End\0"
//		"CapsLock\0"
//		"ScrollLock\0"
//		"NumLock\0"
//		"PrintScreen\0"
//		"Pause\0"
//		"F1\0"
//		"F2\0"
//		"F3\0"
//		"F4\0"
//		"F5\0"
//		"F6\0"
//		"F7\0"
//		"F8\0"
//		"F9\0"
//		"F10\0"
//		"F11\0"
//		"F12\0"
//		"F13\0"
//		"F14\0"
//		"F15\0"
//		"F16\0"
//		"F17\0"
//		"F18\0"
//		"F19\0"
//		"F20\0"
//		"F21\0"
//		"F22\0"
//		"F23\0"
//		"F24\0"
//		"F25\0"
//		"KP_0\0"
//		"KP_1\0"
//		"KP_2\0"
//		"KP_3\0"
//		"KP_4\0"
//		"KP_5\0"
//		"KP_6\0"
//		"KP_7\0"
//		"KP_8\0"
//		"KP_9\0"
//		"KP_DECIMAL\0"
//		"KP_DIVIDE\0"
//		"KP_MULTIPLY\0"
//		"KP_SUBTRACT\0"
//		"KP_ADD\0"
//		"KP_ENTER\0"
//		"KP_EQUAL\0"
//		"LeftShift\0"
//		"LeftControl\0"
//		"LeftAlt\0"
//		"LeftSuper\0"
//		"RightShift\0"
//		"RightControl\0"
//		"RightAlt\0"
//		"RightSuper\0"
//		"Menu\0"
//		"\0";
//
//
//
//		static const char* GetIKeyCodeName(IKeyCode keycode)
//		{
//			const int idx = int(keycode);
//			const char* items_separated_by_zeros = (const char*)IKeyCodeNames;
//			int items_count = 0;
//			const char* p = items_separated_by_zeros;
//			while (*p)
//			{
//				if (idx == items_count)
//					break;
//				p += strlen(p) + 1;
//				items_count++;
//			}
//			return *p ? p : nullptr;
//		}


}//# InputSystem namespace

