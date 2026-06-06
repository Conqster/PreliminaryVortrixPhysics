#pragma once

#include <vector>
#include <string>


struct ResourceDebug
{
	std::string name = "Unknown";
};


class Texture;

template<typename AssetType>
class ResourceRegistry
{
	using Idx = uint32_t;
	static constexpr Idx kInvalidIdx = 0xffffffff;
public: 
	static ResourceRegistry& Instance()
	{
		static ResourceRegistry inst;
		return inst;
	}

	Idx Register(AssetType* asset)
	{
		mAssets.push_back(asset);
		mAssetsName.push_back({});
		return (mAssets.size() - 1);
	}

	void Unregister(AssetType* assets);

	void SetAssetName(Idx idx, const std::string& name) { mAssetsName[idx].name = name; }
	std::string_view GetAssetName(Idx idx) { return mAssetsName[idx].name; }

	const std::vector<AssetType*>& GetAll() const
	{
		return mAssets;
	}
private:
	friend Texture;

	std::vector<AssetType*> mAssets;
	std::vector<ResourceDebug> mAssetsName;
};


using TextureRegistry = ResourceRegistry<Texture>;
template class ResourceRegistry<Texture>;