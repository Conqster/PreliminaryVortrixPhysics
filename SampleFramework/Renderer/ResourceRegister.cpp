#include "ResourceRegister.h"

#include "Texture.h"

template<typename AssetType>
void ResourceRegistry<AssetType>::Unregister(AssetType* assets)
{
	//for (auto it = mAssets.begin(); it != mAssets.end(); ++it)
	//{
	//	if (*it == assets)
	//	{
	//		mAssets.erase(it);
	//		return;
	//	}
	//}


	Idx idx = assets->mRegisterIdx;
	Idx idx_end = mAssets.size() - 1;

	//swap 
	if (idx < idx_end)
	{
		std::swap(mAssets[idx], mAssets[idx_end]);
		mAssets[idx]->mRegisterIdx = idx;
	}

	assets->mRegisterIdx = kInvalidIdx;
	mAssets.pop_back();
}
