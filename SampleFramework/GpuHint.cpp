


/// force device (or devices) to use main graphics card 
/// not intergrated 
extern "C" {
	/*__declspec(dllexport) */unsigned long NvOptimusEnablement = 0x00000001;
	/*__declspec(dllexport)*/ int AmdPowerXpressRequestHighPerformance = 1;
}