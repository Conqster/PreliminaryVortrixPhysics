#pragma once

#include <vector>

template<typename AssetType>
class ResourceRegistry
{
public: 
	static ResourceRegistry& Instance()
	{
		static ResourceRegistry inst;
		return inst;
	}

	void Register(AssetType* asset)
	{
		mAssets.push_back(asset);
	}

	void Unregister(AssetType* assets)
	{
		for (auto it = mAssets.begin(); it != mAssets.end(); ++it)
		{
			if (*it == assets)
			{
				mAssets.erase(it);
				return;
			}
		}
	}

	const std::vector<AssetType*>& GetAll() const
	{
		return mAssets;
	}
private:
	std::vector<AssetType*> mAssets;
};


class Texture;
using TextureRegistry = ResourceRegistry<Texture>;