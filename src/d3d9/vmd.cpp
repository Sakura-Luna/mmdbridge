#include "vmd.h"

#ifdef WITH_VMD

#include "d3d9.h"

#include <shlwapi.h>

#include "bridge_parameter.h"
#include "UMStringUtil.h"
#include "UMPath.h"

#include <cmath>
#include <array>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <map>
#include <set>
#include <sstream>
#include <vector>
#include <algorithm>
#include <unordered_map>

#include <pybind11/pybind11.h>
namespace py = pybind11;

#include <ImathMatrix.h>
#include <ImathMatrixAlgo.h>
#include <ImathQuat.h>
#include <ImathVec.h>

#include <EncodingHelper.h>
#include <Pmd.h>
#include <Pmx.h>
#include <Vmd.h>

template <class T>
std::string to_string(T value)
{
	return umbase::UMStringUtil::number_to_string(value);
}

typedef std::shared_ptr<pmd::PmdModel> PMDPtr;
typedef std::shared_ptr<pmx::PmxModel> PMXPtr;
typedef std::shared_ptr<vmd::VmdMotion> VMDPtr;

static void ShowFrameRangeConfigError()
{
	MessageBoxW(NULL,
				L"Invalid frame range settings detected.\n\n"
				L"Please ensure that:\n"
				L"AVI Export Start Frame <= MMDBridge Start Frame",
				L"Error",
				MB_OK | MB_SETFOREGROUND | MB_ICONERROR);
}

static void ShowInvalidBoneNameError()
{
	MessageBoxW(NULL,
				L"Invalid bone name detected.\n\n"
				L"Bone names must comply with CP932 encoding and cannot exceed 15 bytes. "
				L"Please use tools such as Blender MMD Tools to fix the bone names.\n\n"
				L"For more information, please refer to the tutorials:\n"
				L"https://github.com/rintrint/mmdbridge/blob/master/docs/how_to_use.md\n"
				L"https://www.bilibili.com/opus/1102730546871533640\n\n"
				L"(Press Ctrl+C to copy this message)",
				L"Error",
				MB_OK | MB_SETFOREGROUND | MB_ICONERROR);
}

class FileDataForVMD
{
public:
	FileDataForVMD() = default;
	~FileDataForVMD() = default;

	VMDPtr vmd;
	PMDPtr pmd;
	PMXPtr pmx;
	std::map<int, int> parent_index_map;
	std::map<int, std::string> bone_name_map;
	std::map<int, int> physics_bone_map;
	std::map<int, int> ik_bone_map;
	std::map<int, int> ik_frame_bone_map;
	std::map<int, int> fuyo_bone_map;
	std::map<int, int> fuyo_target_map;
	std::unordered_map<int, vmd::VmdBoneFrame> last_bone_frame;
	// morph (face)
	std::map<int, std::string> morph_name_map;

	FileDataForVMD(const FileDataForVMD& data)
	{
		this->vmd = data.vmd;
		this->pmd = data.pmd;
		this->pmx = data.pmx;
		this->parent_index_map = data.parent_index_map;
		this->bone_name_map = data.bone_name_map;
		this->physics_bone_map = data.physics_bone_map;
		this->ik_bone_map = data.ik_bone_map;
		this->ik_frame_bone_map = data.ik_frame_bone_map;
		this->fuyo_bone_map = data.fuyo_bone_map;
		this->fuyo_target_map = data.fuyo_target_map;
		// morph (face)
		this->morph_name_map = data.morph_name_map;
	}
};

class VMDArchive
{
public:
	static VMDArchive& instance()
	{
		static VMDArchive instance;
		return instance;
	}

	std::vector<FileDataForVMD> data_list;
	std::wstring output_path;
	bool has_bone_name_error = false;
	bool is_start_vmd_export_called = false;
	bool is_start_vmd_export_warning_shown = false;

	// export settings
	int export_fk_bone_animation_mode = 1;
	bool export_ik_bone_animation = false;
	bool add_turn_off_ik_keyframe = true;
	bool export_morph_animation = true;
	bool export_vertex_morph_animation_only = false;

	void end()
	{
		data_list.clear();
		output_path.clear();
		has_bone_name_error = false;
		is_start_vmd_export_called = false;
		is_start_vmd_export_warning_shown = false;
	}

	~VMDArchive() = default;

private:
	VMDArchive() = default;
};

static bool start_vmd_export(
	const int export_fk_bone_animation_mode,
	const bool export_ik_bone_animation,
	const bool add_turn_off_ik_keyframe,
	const bool export_morph_animation,
	const bool export_vertex_morph_animation_only)
{
	BridgeParameter::mutable_instance().current_export_type = ExportType::VMD;

	// Clear previous export to ensure a clean state
	VMDArchive::instance().end();

	VMDArchive& archive = VMDArchive::instance();
	archive.is_start_vmd_export_called = true;
	const BridgeParameter& parameter = BridgeParameter::instance();
	if (parameter.export_fps <= 0)
	{
		return false;
	}

	std::wstring output_path = parameter.base_path + L"out/";

	// Make sure the output folder exists.
	if (!CreateDirectoryW(output_path.c_str(), NULL) && GetLastError() != ERROR_ALREADY_EXISTS)
	{
		std::wstring error_message = L"Cannot create output folder: " + output_path;
		::MessageBoxW(NULL, error_message.c_str(), L"Error", MB_OK | MB_SETFOREGROUND | MB_ICONERROR);
	}

	VMDArchive::instance().output_path = output_path;

	archive.export_fk_bone_animation_mode = export_fk_bone_animation_mode;
	archive.export_ik_bone_animation = export_ik_bone_animation;
	archive.add_turn_off_ik_keyframe = add_turn_off_ik_keyframe;
	archive.export_morph_animation = export_morph_animation;
	archive.export_vertex_morph_animation_only = export_vertex_morph_animation_only;

	const int pmd_num = ExpGetPmdNum();
	for (int i = 0; i < pmd_num; ++i)
	{
		const char* filepath = ExpGetPmdFilenameUtf8(i);
		std::wstring filepath_wstring = umbase::UMStringUtil::utf16_to_wstring(umbase::UMStringUtil::utf8_to_utf16(filepath));
		std::wstring file_ext_wstring = PathFindExtensionW(filepath_wstring.c_str());
		std::transform(file_ext_wstring.begin(), file_ext_wstring.end(), file_ext_wstring.begin(), ::towlower);

		if (file_ext_wstring == L".pmd")
		{
			PMDPtr pmd;
			if ((pmd = pmd::PmdModel::LoadFromFile(filepath)))
			{
			}
			else
			{
				std::wstring error_message = L"Failed to load pmd file: " + filepath_wstring;
				::MessageBoxW(NULL, error_message.c_str(), L"Error", MB_OK | MB_SETFOREGROUND | MB_ICONERROR);
			}
			FileDataForVMD data;
			data.pmd = pmd;
			archive.data_list.push_back(data);
		}
		else if (file_ext_wstring == L".pmx")
		{
			const auto pmx = std::make_shared<pmx::PmxModel>();
			std::ifstream stream(filepath_wstring, std::ios_base::binary);
			if (stream.good())
			{
				pmx->Init();
				pmx->Read(&stream);
			}
			else
			{
				std::wstring error_message = L"Failed to open pmx file: " + filepath_wstring;
				::MessageBoxW(NULL, error_message.c_str(), L"Error", MB_OK | MB_SETFOREGROUND | MB_ICONERROR);
			}
			FileDataForVMD data;
			data.pmx = pmx;
			archive.data_list.push_back(data);
		}
		else
		{
			std::wstring error_message;
			if (filepath_wstring.empty())
			{
				error_message = L"Unable to get pmd/pmx filepath.";
			}
			else
			{
				error_message = L"This is not a pmd/pmx file: " + filepath_wstring;
			}
			::MessageBoxW(NULL, error_message.c_str(), L"Error", MB_OK | MB_SETFOREGROUND | MB_ICONERROR);
		}
	}

	return true;
}

/**
 * @brief 通用的關鍵幀後處理與過濾輔助函數
 * @tparam T 幀的類型
 * @tparam GetNameFunc 獲取名稱的函式類型
 * @tparam AreEqualFunc 比較相等的函式類型
 * @tparam IsZeroFunc 判斷是否為零值的函式類型
 */
template <typename T, typename GetNameFunc, typename AreEqualFunc, typename IsZeroFunc>
std::vector<T> PostProcessKeyframes(
	const std::vector<T>& all_frames,
	GetNameFunc get_name_func,
	AreEqualFunc are_equal_func,
	IsZeroFunc is_zero_func)
{
	if (all_frames.empty())
	{
		return {};
	}

	// 將所有幀按其名稱分組
	std::map<std::string, std::vector<T>> grouped_frames;
	for (const auto& frame : all_frames)
	{
		grouped_frames[get_name_func(frame)].push_back(frame);
	}

	std::vector<T> final_frames;
	final_frames.reserve(all_frames.size());

	// 對每一組 (每一個骨骼或表情) 獨立進行過濾
	for (auto const& [name, frames] : grouped_frames)
	{
		if (frames.empty())
		{
			continue;
		}

		// ------------------------- 過濾規則實施開始 -------------------------
		// 規則 1: 先清除相同的中間幀
		std::vector<T> stage1_frames;
		if (frames.size() > 2)
		{
			stage1_frames.push_back(frames.front()); // 總是保留第一幀
			for (int i = 1; i < frames.size() - 1; ++i)
			{
				// 如果一個幀和它前後的幀都相等，它就是可移除的中間幀
				if (!(are_equal_func(frames[i - 1], frames[i]) && are_equal_func(frames[i], frames[i + 1])))
				{
					stage1_frames.push_back(frames[i]);
				}
			}
			stage1_frames.push_back(frames.back()); // 總是保留最後一幀
		}
		else
		{
			stage1_frames = frames; // 幀數小於等於2，沒有中間幀
		}

		// 規則 2: 如果只剩下頭尾且頭尾一樣, 留下頭
		std::vector<T> stage2_frames;
		if (stage1_frames.size() == 2 && are_equal_func(stage1_frames[0], stage1_frames[1]))
		{
			stage2_frames.push_back(stage1_frames[0]);
		}
		else
		{
			stage2_frames = stage1_frames;
		}

		// 規則 3: 如果只剩下頭 且頭為0.0 直接刪除
		if (stage2_frames.size() == 1 && is_zero_func(stage2_frames[0]))
		{
			// 不做任何事，即刪除這個表情的所有關鍵幀
		}
		else
		{
			// 將最終結果合併到 final_frames
			final_frames.insert(final_frames.end(), stage2_frames.begin(), stage2_frames.end());
		}
		// ------------------------- 過濾規則實施結束 -------------------------
	}

	// 最終結果按幀號排序
	std::sort(final_frames.begin(), final_frames.end(),
			  [](const T& a, const T& b) {
				  return a.frame < b.frame;
			  });

	return final_frames;
}

struct GltfBinaryData
{
	std::vector<unsigned char> data;
	std::vector<std::string> buffer_views;
	std::vector<std::string> accessors;

	void align()
	{
		while (data.size() % 4 != 0)
		{
			data.push_back(0);
		}
	}

	int add_accessor(
		const void* source,
		const size_t byte_length,
		const int component_type,
		const size_t count,
		const char* type,
		const int target)
	{
		align();
		const size_t byte_offset = data.size();
		const auto* bytes = static_cast<const unsigned char*>(source);
		data.insert(data.end(), bytes, bytes + byte_length);

		std::ostringstream buffer_view;
		buffer_view << "{\"buffer\":0,\"byteOffset\":" << byte_offset
					<< ",\"byteLength\":" << byte_length;
		if (target != 0)
		{
			buffer_view << ",\"target\":" << target;
		}
		buffer_view << "}";
		const int buffer_view_index = static_cast<int>(buffer_views.size());
		buffer_views.push_back(buffer_view.str());

		std::ostringstream accessor;
		accessor << "{\"bufferView\":" << buffer_view_index
				 << ",\"componentType\":" << component_type
				 << ",\"count\":" << count
				 << ",\"type\":\"" << type << "\"";
		accessor << "}";
		const int accessor_index = static_cast<int>(accessors.size());
		accessors.push_back(accessor.str());
		return accessor_index;
	}
};

struct GltfMorphData
{
	std::string name;
	std::string animation_name;
	std::vector<float> position_deltas;
};

struct GltfModelData
{
	std::string name;
	std::vector<float> positions;
	std::vector<float> normals;
	std::vector<float> texcoords;
	std::vector<uint16_t> joints;
	std::vector<float> weights;
	std::vector<uint32_t> indices;
	std::vector<int> material_index_counts;
	std::vector<std::array<float, 4>> material_colors;
	std::vector<std::string> material_names;
	std::vector<std::string> bone_names;
	std::vector<std::string> bone_animation_names;
	std::vector<std::array<float, 3>> bone_positions;
	std::vector<int> bone_parents;
	std::vector<GltfMorphData> morphs;
};

static std::string gltf_json_string(const std::string &value) {
	std::ostringstream result;
	result << '"';
	for (const unsigned char character: value) {
		switch (character) {
			case '"': result << "\\\"";
				break;
			case '\\': result << "\\\\";
				break;
			case '\b': result << "\\b";
				break;
			case '\f': result << "\\f";
				break;
			case '\n': result << "\\n";
				break;
			case '\r': result << "\\r";
				break;
			case '\t': result << "\\t";
				break;
			default: if (character < 0x20) {
					result << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
						<< static_cast<int>(character) << std::dec;
				} else {
					result << static_cast<char>(character);
				}
				break;
		}
	}
	result << '"';
	return result.str();
}

static std::string gltf_json_float(const float value) {
	if (!std::isfinite(value)) {
		return "0";
	}
	std::ostringstream result;
	result << std::setprecision(9) << value;
	return result.str();
}

static std::string gltf_json_float_array(const float *values, const size_t count) {
	std::ostringstream result;
	result << '[';
	for (size_t i = 0; i < count; ++i) {
		if (i != 0) {
			result << ',';
		}
		result << gltf_json_float(values[i]);
	}
	result << ']';
	return result.str();
}

static std::string gltf_cp932_to_utf8(const std::string &value) {
	if (value.empty()) {
		return {};
	}
	std::string result;
	oguna::EncodingConverter::Cp932ToUtf8(value.c_str(), static_cast<int>(value.size()), &result);
	return result;
}

static std::string gltf_utf16_to_utf8(const std::wstring &value) {
	return umbase::UMStringUtil::wstring_to_utf8(value);
}

static void gltf_add_pmx_vertex_morph(
	const pmx::PmxModel &model,
	const int morph_index,
	const float weight,
	std::vector<float> &deltas,
	std::set<int> &visiting) {
	if (morph_index < 0 || morph_index >= static_cast<int>(model.morphs.size()) || weight == 0.0f ||
	    visiting.find(morph_index) != visiting.end()) {
		return;
	}

	const pmx::PmxMorph &morph = model.morphs[morph_index];
	visiting.insert(morph_index);
	if (morph.morph_type == pmx::MorphType::Vertex) {
		for (const auto &offset: morph.vertex_offsets) {
			if (offset.vertex_index < 0 || offset.vertex_index >= static_cast<int>(model.vertices.size())) {
				continue;
			}
			const size_t base = static_cast<size_t>(offset.vertex_index) * 3;
			for (int i = 0; i < 3; ++i) {
				deltas[base + i] += offset.position_offset[i] * weight;
			}
		}
	} else if (morph.morph_type == pmx::MorphType::Group) {
		for (const auto &offset: morph.group_offsets) {
			gltf_add_pmx_vertex_morph(model, offset.morph_index, weight * offset.morph_weight, deltas, visiting);
		}
	}
	visiting.erase(morph_index);
}

static GltfModelData build_gltf_model_data(const FileDataForVMD &file_data) {
	GltfModelData result;

	if (file_data.pmd) {
		const pmd::PmdModel &model = *file_data.pmd;
		result.name = gltf_cp932_to_utf8(model.header.name);
		result.positions.reserve(model.vertices.size() * 3);
		result.normals.reserve(model.vertices.size() * 3);
		result.texcoords.reserve(model.vertices.size() * 2);
		result.joints.reserve(model.vertices.size() * 4);
		result.weights.reserve(model.vertices.size() * 4);
		for (const auto &vertex: model.vertices) {
			for (int i = 0; i < 3; ++i) {
				result.positions.push_back(vertex.position[i]);
				result.normals.push_back(vertex.normal[i]);
			}
			result.texcoords.push_back(vertex.uv[0]);
			result.texcoords.push_back(vertex.uv[1]);

			const uint16_t first_bone = vertex.bone_index[0];
			const uint16_t second_bone = vertex.bone_index[1];
			const float first_weight = static_cast<float>(vertex.bone_weight) / 100.0f;
			result.joints.insert(result.joints.end(), {first_bone, second_bone, 0, 0});
			result.weights.insert(result.weights.end(), {first_weight, 1.0f - first_weight, 0.0f, 0.0f});
		}
		for (const uint16_t index: model.indices) {
			result.indices.push_back(index);
		}
		for (const auto &material: model.materials) {
			result.material_index_counts.push_back(static_cast<int>(material.index_count));
			result.material_colors.push_back(
				{material.diffuse[0], material.diffuse[1], material.diffuse[2], material.diffuse[3]});
			result.material_names.push_back("Material " + std::to_string(result.material_names.size()));
		}
		for (const auto &bone: model.bones) {
			result.bone_names.push_back(gltf_cp932_to_utf8(bone.name));
			result.bone_animation_names.push_back(bone.name);
			result.bone_positions.push_back({bone.bone_head_pos[0], bone.bone_head_pos[1], bone.bone_head_pos[2]});
			result.bone_parents.push_back(bone.parent_bone_index == 0xFFFF ? -1 : bone.parent_bone_index);
		}
		for (size_t face_index = 1; face_index < model.faces.size(); ++face_index) {
			const auto &face = model.faces[face_index];
			GltfMorphData morph;
			morph.name = gltf_cp932_to_utf8(face.name);
			morph.animation_name = face.name;
			morph.position_deltas.resize(model.vertices.size() * 3, 0.0f);
			for (const auto &vertex: face.vertices) {
				if (vertex.vertex_index < 0 || vertex.vertex_index >= static_cast<int>(model.vertices.size())) {
					continue;
				}
				const size_t base = static_cast<size_t>(vertex.vertex_index) * 3;
				for (int i = 0; i < 3; ++i) {
					morph.position_deltas[base + i] =
						vertex.position[i] - model.vertices[vertex.vertex_index].position[i];
				}
			}
			result.morphs.push_back(std::move(morph));
		}
	} else if (file_data.pmx) {
		const pmx::PmxModel &model = *file_data.pmx;
		result.name = gltf_utf16_to_utf8(model.model_name);
		result.positions.reserve(model.vertices.size() * 3);
		result.normals.reserve(model.vertices.size() * 3);
		result.texcoords.reserve(model.vertices.size() * 2);
		result.joints.reserve(model.vertices.size() * 4);
		result.weights.reserve(model.vertices.size() * 4);
		for (const auto &vertex: model.vertices) {
			for (int i = 0; i < 3; ++i) {
				result.positions.push_back(vertex.position[i]);
				result.normals.push_back(vertex.normal[i]);
			}
			result.texcoords.push_back(vertex.uv[0]);
			result.texcoords.push_back(vertex.uv[1]);

			std::array<uint16_t, 4> joints = {0, 0, 0, 0};
			std::array < float, 4 > weights = {0.0f, 0.0f, 0.0f, 0.0f};
			auto set_weight = [&](const int slot, const int bone_index, const float weight) {
				if (slot >= 0 && slot < 4 && bone_index >= 0 && bone_index < static_cast<int>(model.bones.size())) {
					joints[slot] = static_cast<uint16_t>(bone_index);
					weights[slot] = weight;
				}
			};
			switch (vertex.skinning_type) {
				case pmx::PmxVertexSkinningType::BDEF1: {
					const auto *skinning = dynamic_cast<const pmx::PmxVertexSkinningBDEF1 *>(vertex.skinning.get());
					if (skinning) {
						set_weight(0, skinning->bone_index, 1.0f);
					}
					break;
				}
				case pmx::PmxVertexSkinningType::BDEF2:
				case pmx::PmxVertexSkinningType::SDEF: {
					const auto *skinning = dynamic_cast<const pmx::PmxVertexSkinningBDEF2 *>(vertex.skinning.get());
					const auto *sdef = dynamic_cast<const pmx::PmxVertexSkinningSDEF *>(vertex.skinning.get());
					if (skinning) {
						set_weight(0, skinning->bone_index1, skinning->bone_weight);
						set_weight(1, skinning->bone_index2, 1.0f - skinning->bone_weight);
					} else if (sdef) {
						set_weight(0, sdef->bone_index1, sdef->bone_weight);
						set_weight(1, sdef->bone_index2, 1.0f - sdef->bone_weight);
					}
					break;
				}
				case pmx::PmxVertexSkinningType::BDEF4:
				case pmx::PmxVertexSkinningType::QDEF: {
					const auto *skinning = dynamic_cast<const pmx::PmxVertexSkinningBDEF4 *>(vertex.skinning.get());
					const auto *qdef = dynamic_cast<const pmx::PmxVertexSkinningQDEF *>(vertex.skinning.get());
					if (skinning) {
						set_weight(0, skinning->bone_index1, skinning->bone_weight1);
						set_weight(1, skinning->bone_index2, skinning->bone_weight2);
						set_weight(2, skinning->bone_index3, skinning->bone_weight3);
						set_weight(3, skinning->bone_index4, skinning->bone_weight4);
					} else if (qdef) {
						set_weight(0, qdef->bone_index1, qdef->bone_weight1);
						set_weight(1, qdef->bone_index2, qdef->bone_weight2);
						set_weight(2, qdef->bone_index3, qdef->bone_weight3);
						set_weight(3, qdef->bone_index4, qdef->bone_weight4);
					}
					break;
				}
				default:
					break;
			}
			float weight_sum = 0.0f;
			for (const float weight: weights) {
				weight_sum += weight;
			}
			if (weight_sum > 0.0f) {
				for (float &weight: weights) {
					weight /= weight_sum;
				}
			}
			result.joints.insert(result.joints.end(), joints.begin(), joints.end());
			result.weights.insert(result.weights.end(), weights.begin(), weights.end());
		}
		for (const int index: model.indices) {
			if (index >= 0) {
				result.indices.push_back(static_cast<uint32_t>(index));
			}
		}
		for (const auto &material: model.materials) {
			result.material_index_counts.push_back(material.index_count);
			result.material_colors.push_back(
				{material.diffuse[0], material.diffuse[1], material.diffuse[2], material.diffuse[3]});
			result.material_names.push_back(gltf_utf16_to_utf8(material.material_name));
		}
		for (const auto &bone: model.bones) {
			result.bone_names.push_back(gltf_utf16_to_utf8(bone.bone_name));
			std::string animation_name;
			oguna::EncodingConverter::Utf16ToCp932(
				bone.bone_name.c_str(), static_cast<int>(bone.bone_name.length()), &animation_name);
			result.bone_animation_names.push_back(animation_name);
			result.bone_positions.push_back({bone.position[0], bone.position[1], bone.position[2]});
			result.bone_parents.push_back(bone.parent_index);
		}
		for (size_t morph_index = 0; morph_index < model.morphs.size(); ++morph_index) {
			GltfMorphData morph;
			morph.name = gltf_utf16_to_utf8(model.morphs[morph_index].morph_name);
			auto animation_name = file_data.morph_name_map.find(static_cast<int>(morph_index));
			morph.animation_name = animation_name != file_data.morph_name_map.end() ?
				animation_name->second :
				morph.name;
			morph.position_deltas.resize(model.vertices.size() * 3, 0.0f);
			std::set<int> visiting;
			gltf_add_pmx_vertex_morph(model, static_cast<int>(morph_index), 1.0f, morph.position_deltas, visiting);
			bool has_delta = false;
			for (const float delta: morph.position_deltas) {
				if (std::abs(delta) > 1e-8f) {
					has_delta = true;
					break;
				}
			}
			if (has_delta) {
				result.morphs.push_back(std::move(morph));
			}
		}
	}

	return result;
}

static std::string gltf_join_json(const std::vector<std::string> &values) {
	std::ostringstream result;
	for (size_t i = 0; i < values.size(); ++i) {
		if (i != 0) {
			result << ',';
		}
		result << values[i];
	}
	return result.str();
}

static std::string gltf_morph_weight_type(const size_t morph_count) {
	if (morph_count == 1) {
		return "SCALAR";
	}
	return "VEC" + std::to_string(morph_count);
}

static bool write_gltf_file(
	const FileDataForVMD &file_data,
	const std::wstring &gltf_filepath,
	const std::wstring &bin_filepath) {
	GltfModelData model = build_gltf_model_data(file_data);
	if (model.positions.empty() || model.indices.empty()) {
		::MessageBoxW(NULL, L"Cannot export an empty PMD/PMX model as glTF.", L"Error",
		              MB_OK | MB_SETFOREGROUND | MB_ICONERROR);
		return false;
	}
	const double export_fps = BridgeParameter::instance().export_fps;
	if (!(export_fps > 0.0)) {
		::MessageBoxW(NULL, L"Cannot export glTF with an invalid frame rate.", L"Error",
		              MB_OK | MB_SETFOREGROUND | MB_ICONERROR);
		return false;
	}

	GltfBinaryData binary;
	const size_t vertex_count = model.positions.size() / 3;
	const int position_accessor = binary.add_accessor(
		model.positions.data(), model.positions.size() * sizeof(float), 5126, vertex_count, "VEC3", 34962);
	const int normal_accessor = binary.add_accessor(
		model.normals.data(), model.normals.size() * sizeof(float), 5126, vertex_count, "VEC3", 34962);
	const int texcoord_accessor = binary.add_accessor(
		model.texcoords.data(), model.texcoords.size() * sizeof(float), 5126, vertex_count, "VEC2", 34962);
	const bool has_skin = !model.bone_names.empty();
	const int joints_accessor = has_skin ?
		binary.add_accessor(model.joints.data(), model.joints.size() * sizeof(uint16_t),
		                    5123, vertex_count, "VEC4", 34962) :
		-1;
	const int weights_accessor = has_skin ?
		binary.add_accessor(model.weights.data(), model.weights.size() * sizeof(float),
		                    5126, vertex_count, "VEC4", 34962) :
		-1;

	std::vector<int> morph_accessors;
	morph_accessors.reserve(model.morphs.size());
	for (const auto &morph: model.morphs) {
		morph_accessors.push_back(binary.add_accessor(
			morph.position_deltas.data(), morph.position_deltas.size() * sizeof(float), 5126, vertex_count, "VEC3",
			34962));
	}

	std::vector<std::string> material_json;
	for (size_t i = 0; i < model.material_colors.size(); ++i) {
		const auto &color = model.material_colors[i];
		std::ostringstream material;
		material << "{\"name\":" << gltf_json_string(model.material_names[i])
			<< ",\"pbrMetallicRoughness\":{\"baseColorFactor\":"
			<< gltf_json_float_array(color.data(), color.size())
			<< ",\"metallicFactor\":0,\"roughnessFactor\":1},\"doubleSided\":true}";
		material_json.push_back(material.str());
	}

	std::vector<std::string> primitive_json;
	const auto make_primitive = [&](const int index_accessor, const int material_index) {
		std::ostringstream primitive;
		primitive << "{\"attributes\":{\"POSITION\":" << position_accessor
			<< ",\"NORMAL\":" << normal_accessor
			<< ",\"TEXCOORD_0\":" << texcoord_accessor;
		if (has_skin) {
			primitive << ",\"JOINTS_0\":" << joints_accessor << ",\"WEIGHTS_0\":" << weights_accessor;
		}
		primitive << "},\"indices\":" << index_accessor << ",\"mode\":4";
		if (material_index >= 0) {
			primitive << ",\"material\":" << material_index;
		}
		if (!model.morphs.empty()) {
			primitive << ",\"targets\":[";
			for (size_t morph_index = 0; morph_index < morph_accessors.size(); ++morph_index) {
				if (morph_index != 0) {
					primitive << ',';
				}
				primitive << "{\"POSITION\":" << morph_accessors[morph_index] << "}";
			}
			primitive << ']';
		}
		primitive << '}';
		return primitive.str();
	};

	size_t index_offset = 0;
	for (size_t material_index = 0; material_index < model.material_index_counts.size(); ++material_index) {
		const size_t material_index_count =
			static_cast<size_t>(std::max(0, model.material_index_counts[material_index]));
		const size_t available_index_count = model.indices.size() - std::min(index_offset, model.indices.size());
		const size_t material_offset = index_offset;
		const size_t consumed_index_count = std::min(material_index_count, available_index_count);
		const size_t index_count = consumed_index_count - consumed_index_count % 3;
		index_offset += consumed_index_count;
		if (index_count == 0) {
			continue;
		}
		std::vector<uint32_t> material_indices(
			model.indices.begin() + static_cast<std::ptrdiff_t>(material_offset),
			model.indices.begin() + static_cast<std::ptrdiff_t>(material_offset + index_count));
		const int index_accessor = binary.add_accessor(
			material_indices.data(), material_indices.size() * sizeof(uint32_t), 5125, material_indices.size(),
			"SCALAR", 34963);
		primitive_json.push_back(make_primitive(
			index_accessor, material_index < material_json.size() ? static_cast<int>(material_index) : -1));
	}
	if (model.material_index_counts.empty()) {
		const size_t index_count = model.indices.size() - model.indices.size() % 3;
		if (index_count > 0) {
			const int index_accessor = binary.add_accessor(
				model.indices.data(), index_count * sizeof(uint32_t), 5125, index_count, "SCALAR", 34963);
			primitive_json.push_back(make_primitive(index_accessor, -1));
		}
	}
	if (primitive_json.empty()) {
		::MessageBoxW(NULL, L"Cannot export PMD/PMX primitives as glTF.", L"Error",
		              MB_OK | MB_SETFOREGROUND | MB_ICONERROR);
		return false;
	}

	std::ostringstream mesh;
	mesh << "{\"name\":" << gltf_json_string(model.name)
		<< ",\"primitives\":[" << gltf_join_json(primitive_json) << ']';
	if (!model.morphs.empty()) {
		mesh << ",\"weights\":[";
		for (size_t i = 0; i < model.morphs.size(); ++i) {
			if (i != 0) {
				mesh << ',';
			}
			mesh << "0";
		}
		mesh << "],\"extras\":{\"targetNames\":[";
		for (size_t i = 0; i < model.morphs.size(); ++i) {
			if (i != 0) {
				mesh << ',';
			}
			mesh << gltf_json_string(model.morphs[i].name);
		}
		mesh << "]}}";
	} else {
		mesh << '}';
	}

	int inverse_bind_accessor = -1;
	if (has_skin) {
		std::vector<float> inverse_bind_matrices;
		inverse_bind_matrices.reserve(model.bone_positions.size() * 16);
		for (const auto &position: model.bone_positions) {
			inverse_bind_matrices.insert(
				inverse_bind_matrices.end(),
				{
					1.0f, 0.0f, 0.0f, 0.0f,
					0.0f, 1.0f, 0.0f, 0.0f,
					0.0f, 0.0f, 1.0f, 0.0f,
					-position[0], -position[1], -position[2], 1.0f
				});
		}
		inverse_bind_accessor = binary.add_accessor(
			inverse_bind_matrices.data(), inverse_bind_matrices.size() * sizeof(float), 5126,
			model.bone_positions.size(), "MAT4", 0);
	}

	std::vector<std::vector<int> > children(model.bone_names.size());
	std::vector<int> root_bones;
	for (size_t bone_index = 0; bone_index < model.bone_parents.size(); ++bone_index) {
		const int parent = model.bone_parents[bone_index];
		if (parent >= 0 && parent < static_cast<int>(children.size())) {
			children[parent].push_back(static_cast<int>(bone_index + 1));
		} else {
			root_bones.push_back(static_cast<int>(bone_index + 1));
		}
	}
	if (has_skin && root_bones.empty()) {
		::MessageBoxW(NULL, L"Cannot export a PMD/PMX skeleton without a root bone.", L"Error",
		              MB_OK | MB_SETFOREGROUND | MB_ICONERROR);
		return false;
	}

	std::vector<std::string> node_json;
	{
		std::ostringstream node;
		node << "{\"name\":" << gltf_json_string(model.name) << ",\"mesh\":0";
		if (has_skin) {
			node << ",\"skin\":0";
		}
		if (!root_bones.empty()) {
			node << ",\"children\":[";
			for (size_t i = 0; i < root_bones.size(); ++i) {
				if (i != 0) {
					node << ',';
				}
				node << root_bones[i];
			}
			node << ']';
		}
		node << '}';
		node_json.push_back(node.str());
	}
	for (size_t bone_index = 0; bone_index < model.bone_names.size(); ++bone_index) {
		const int parent = model.bone_parents[bone_index];
		const auto &position = model.bone_positions[bone_index];
		const bool has_parent = parent >= 0 && parent < static_cast<int>(model.bone_positions.size());
		const float local_position[3] = {
			position[0] - (has_parent ? model.bone_positions[parent][0] : 0.0f),
			position[1] - (has_parent ? model.bone_positions[parent][1] : 0.0f),
			position[2] - (has_parent ? model.bone_positions[parent][2] : 0.0f)
		};
		std::ostringstream node;
		node << "{\"name\":" << gltf_json_string(model.bone_names[bone_index])
			<< ",\"translation\":" << gltf_json_float_array(local_position, 3);
		if (!children[bone_index].empty()) {
			node << ",\"children\":[";
			for (size_t i = 0; i < children[bone_index].size(); ++i) {
				if (i != 0) {
					node << ',';
				}
				node << children[bone_index][i];
			}
			node << ']';
		}
		node << '}';
		node_json.push_back(node.str());
	}

	std::vector<std::string> animation_samplers;
	std::vector<std::string> animation_channels;
	bool uses_animation_pointer = false;
	const int start_frame = BridgeParameter::instance().start_frame;
	const auto frame_time = [&](const int frame) {
		return static_cast<float>(std::max(0, frame - start_frame) / export_fps);
	};
	if (file_data.vmd) {
		std::unordered_map<std::string, int> bone_index_by_name;
		for (size_t bone_index = 0; bone_index < model.bone_animation_names.size(); ++bone_index) {
			bone_index_by_name[model.bone_animation_names[bone_index]] = static_cast<int>(bone_index);
		}
		std::map<int, std::vector<const vmd::VmdBoneFrame *> > frames_by_bone;
		for (const auto &frame: file_data.vmd->bone_frames) {
			const auto it = bone_index_by_name.find(frame.name);
			if (it != bone_index_by_name.end()) {
				frames_by_bone[it->second].push_back(&frame);
			}
		}
		for (auto &[bone_index, frames]: frames_by_bone) {
			std::sort(frames.begin(), frames.end(),
			          [](const auto *left, const auto *right) { return left->frame < right->frame; });
			std::vector<float> times;
			std::vector<float> translations;
			std::vector<float> rotations;
			times.reserve(frames.size());
			translations.reserve(frames.size() * 3);
			rotations.reserve(frames.size() * 4);
			const int parent = model.bone_parents[bone_index];
			const auto &rest_position = model.bone_positions[bone_index];
			const bool has_parent = parent >= 0 && parent < static_cast<int>(model.bone_positions.size());
			const float parent_position[3] = {
				has_parent ? model.bone_positions[parent][0] : 0.0f,
				has_parent ? model.bone_positions[parent][1] : 0.0f,
				has_parent ? model.bone_positions[parent][2] : 0.0f
			};
			const float rest_local_position[3] = {
				rest_position[0] - parent_position[0],
				rest_position[1] - parent_position[1],
				rest_position[2] - parent_position[2]
			};
			for (const auto *frame: frames) {
				times.push_back(frame_time(frame->frame));
				for (int i = 0; i < 3; ++i) {
					translations.push_back(rest_local_position[i] + frame->position[i]);
				}
				for (int i = 0; i < 4; ++i) {
					rotations.push_back(frame->orientation[i]);
				}
			}
			const int time_accessor = binary.add_accessor(
				times.data(), times.size() * sizeof(float), 5126, times.size(), "SCALAR", 0);
			const int translation_accessor = binary.add_accessor(
				translations.data(), translations.size() * sizeof(float), 5126, times.size(), "VEC3", 0);
			const int rotation_accessor = binary.add_accessor(
				rotations.data(), rotations.size() * sizeof(float), 5126, times.size(), "VEC4", 0);
			const int translation_sampler = static_cast<int>(animation_samplers.size());
			animation_samplers.push_back(
				"{\"input\":" + std::to_string(time_accessor) + ",\"output\":" +
				std::to_string(translation_accessor) + ",\"interpolation\":\"LINEAR\"}");
			animation_channels.push_back(
				"{\"sampler\":" + std::to_string(translation_sampler) +
				",\"target\":{\"node\":" + std::to_string(bone_index + 1) + ",\"path\":\"translation\"}}");
			const int rotation_sampler = static_cast<int>(animation_samplers.size());
			animation_samplers.push_back(
				"{\"input\":" + std::to_string(time_accessor) + ",\"output\":" +
				std::to_string(rotation_accessor) + ",\"interpolation\":\"LINEAR\"}");
			animation_channels.push_back(
				"{\"sampler\":" + std::to_string(rotation_sampler) +
				",\"target\":{\"node\":" + std::to_string(bone_index + 1) + ",\"path\":\"rotation\"}}");
		}

		std::unordered_map<std::string, int> morph_index_by_name;
		for (size_t morph_index = 0; morph_index < model.morphs.size(); ++morph_index) {
			morph_index_by_name[model.morphs[morph_index].animation_name] = static_cast<int>(morph_index);
		}
		std::vector<std::map<int, float> > morph_frames(model.morphs.size());
		for (const auto &frame: file_data.vmd->face_frames) {
			const auto it = morph_index_by_name.find(frame.face_name);
			if (it != morph_index_by_name.end()) {
				morph_frames[it->second][frame.frame] = frame.weight;
			}
		}
		std::set<int> morph_times;
		for (const auto &frames: morph_frames) {
			for (const auto &[frame, weight]: frames) {
				(void) weight;
				morph_times.insert(frame);
			}
		}
		if (!morph_times.empty() && model.morphs.size() <= 4) {
			std::vector<float> times;
			std::vector<float> values;
			times.reserve(morph_times.size());
			values.reserve(morph_times.size() * model.morphs.size());
			for (const int frame: morph_times) {
				times.push_back(frame_time(frame));
				for (const auto &frames: morph_frames) {
					const auto value = frames.upper_bound(frame);
					values.push_back(value == frames.begin() ? 0.0f : std::prev(value)->second);
				}
			}
			const int time_accessor = binary.add_accessor(
				times.data(), times.size() * sizeof(float), 5126, times.size(), "SCALAR", 0);
			const std::string weight_type = gltf_morph_weight_type(model.morphs.size());
			const int weight_accessor = binary.add_accessor(
				values.data(), values.size() * sizeof(float), 5126, times.size(), weight_type.c_str(), 0);
			const int sampler_index = static_cast<int>(animation_samplers.size());
			animation_samplers.push_back(
				"{\"input\":" + std::to_string(time_accessor) + ",\"output\":" +
				std::to_string(weight_accessor) + ",\"interpolation\":\"LINEAR\"}");
			animation_channels.push_back(
				"{\"sampler\":" + std::to_string(sampler_index) +
				",\"target\":{\"node\":0,\"path\":\"weights\"}}");
		} else if (!morph_times.empty()) {
			uses_animation_pointer = true;
			for (size_t morph_index = 0; morph_index < morph_frames.size(); ++morph_index) {
				const auto &frames = morph_frames[morph_index];
				if (frames.empty()) {
					continue;
				}
				std::vector<float> times;
				std::vector<float> values;
				for (const auto &[frame, weight]: frames) {
					times.push_back(frame_time(frame));
					values.push_back(weight);
				}
				const int time_accessor = binary.add_accessor(
					times.data(), times.size() * sizeof(float), 5126, times.size(), "SCALAR", 0);
				const int weight_accessor = binary.add_accessor(
					values.data(), values.size() * sizeof(float), 5126, values.size(), "SCALAR", 0);
				const int sampler_index = static_cast<int>(animation_samplers.size());
				animation_samplers.push_back(
					"{\"input\":" + std::to_string(time_accessor) + ",\"output\":" +
					std::to_string(weight_accessor) + ",\"interpolation\":\"LINEAR\"}");
				std::ostringstream channel;
				channel << "{\"sampler\":" << sampler_index
					<< ",\"target\":{\"extensions\":{\"KHR_animation_pointer\":{\"pointer\":\"/nodes/0/weights/"
					<< morph_index << "\"}}}}";
				animation_channels.push_back(channel.str());
			}
		}
	}

	std::ostringstream skin;
	if (has_skin) {
		skin << "{\"inverseBindMatrices\":" << inverse_bind_accessor << ",\"joints\":[";
		for (size_t bone_index = 0; bone_index < model.bone_names.size(); ++bone_index) {
			if (bone_index != 0) {
				skin << ',';
			}
			skin << bone_index + 1;
		}
		skin << "],\"skeleton\":" << root_bones.front() << '}';
	}

	std::ostringstream json;
	json << "{\"asset\":{\"version\":\"2.0\",\"generator\":\"mmdbridge\"},\"scene\":0,\"scenes\":[{\"nodes\":[0]}]"
		<< ",\"nodes\":[" << gltf_join_json(node_json) << "]"
		<< ",\"meshes\":[" << mesh.str() << ']';
	if (!material_json.empty()) {
		json << ",\"materials\":[" << gltf_join_json(material_json) << ']';
	}
	if (has_skin) {
		json << ",\"skins\":[" << skin.str() << ']';
	}
	json << ",\"bufferViews\":[" << gltf_join_json(binary.buffer_views) << "]"
		<< ",\"accessors\":[" << gltf_join_json(binary.accessors) << ']'
		<< ",\"buffers\":[{\"uri\":"
		<< gltf_json_string(
			umbase::UMStringUtil::wstring_to_utf8(std::wstring(PathFindFileNameW(bin_filepath.c_str()))))
		<< ",\"byteLength\":" << binary.data.size() << '}';
	if (!animation_samplers.empty()) {
		json << ",\"animations\":[{\"name\":\"Animation\",\"samplers\":["
			<< gltf_join_json(animation_samplers) << "],\"channels\":["
			<< gltf_join_json(animation_channels) << "]}]";
	}
	if (uses_animation_pointer) {
		json << ",\"extensionsUsed\":[\"KHR_animation_pointer\"]";
	}
	json << '}';

	std::ofstream bin_stream(bin_filepath.c_str(), std::ios::binary);
	if (!bin_stream.good()) {
		::MessageBoxW(NULL, L"Failed to write glTF binary buffer.", L"Error",
		              MB_OK | MB_SETFOREGROUND | MB_ICONERROR);
		return false;
	}
	bin_stream.write(reinterpret_cast<const char *>(binary.data.data()),
	                 static_cast<std::streamsize>(binary.data.size()));
	if (!bin_stream.good()) {
		return false;
	}
	bin_stream.close();

	std::ofstream gltf_stream(gltf_filepath.c_str(), std::ios::binary);
	if (!gltf_stream.good()) {
		::MessageBoxW(NULL, L"Failed to write glTF document.", L"Error",
		              MB_OK | MB_SETFOREGROUND | MB_ICONERROR);
		return false;
	}
	const std::string json_text = json.str();
	gltf_stream.write(json_text.data(), static_cast<std::streamsize>(json_text.size()));
	return gltf_stream.good();
}

static bool end_vmd_export()
{
	VMDArchive& archive = VMDArchive::instance();

	if (!archive.is_start_vmd_export_called)
	{
		archive.end();
		ShowFrameRangeConfigError();
		return false;
	}

	if (archive.has_bone_name_error)
	{
		archive.end();
		ShowInvalidBoneNameError();
		return false;
	}

	const int pmd_num = ExpGetPmdNum();
	std::map<std::wstring, int> output_name_counts;
	for (int i = 0; i < pmd_num; ++i)
	{
		FileDataForVMD& file_data = archive.data_list.at(i);
		if (!file_data.vmd)
		{
			continue;
		}

		// PostProcess
		{
			// morph (face)
			const float face_threshold = 0.0f;
			auto get_face_name = [](const vmd::VmdFaceFrame& f) { return f.face_name; };
			auto are_faces_equal = [&](const vmd::VmdFaceFrame& a, const vmd::VmdFaceFrame& b) {
				return std::abs(a.weight - b.weight) <= face_threshold;
			};
			auto is_face_zero = [&](const vmd::VmdFaceFrame& f) {
				return std::abs(f.weight) <= face_threshold;
			};
			file_data.vmd->face_frames = PostProcessKeyframes(file_data.vmd->face_frames, get_face_name, are_faces_equal, is_face_zero);

			// bone
			const float bone_threshold = 0.0f;
			auto get_bone_name = [](const vmd::VmdBoneFrame& f) { return f.name; };
			auto are_bones_equal = [&](const vmd::VmdBoneFrame& a, const vmd::VmdBoneFrame& b) {
				for (int j = 0; j < 3; ++j)
				{
					if (std::abs(a.position[j] - b.position[j]) > bone_threshold)
						return false;
				}
				for (int j = 0; j < 4; ++j)
				{
					if (std::abs(a.orientation[j] - b.orientation[j]) > bone_threshold)
						return false;
				}
				return true;
			};
			auto is_bone_zero = [&](const vmd::VmdBoneFrame& f) {
				bool position_is_zero = std::abs(f.position[0]) <= bone_threshold &&
										std::abs(f.position[1]) <= bone_threshold &&
										std::abs(f.position[2]) <= bone_threshold;
				bool orientation_is_identity = std::abs(f.orientation[0]) <= bone_threshold &&
											   std::abs(f.orientation[1]) <= bone_threshold &&
											   std::abs(f.orientation[2]) <= bone_threshold &&
											   std::abs(f.orientation[3] - 1.0f) <= bone_threshold;
				return position_is_zero && orientation_is_identity;
			};
			file_data.vmd->bone_frames = PostProcessKeyframes(file_data.vmd->bone_frames, get_bone_name, are_bones_equal, is_bone_zero);
		}

		// Frame Offset Logic: Shift all keyframes forward to make the starting frame become 0
		const int start_frame = BridgeParameter::instance().start_frame;
		if (start_frame > 0)
		{
			// Shift bone keyframes
			for (auto& frame : file_data.vmd->bone_frames)
			{
				frame.frame -= start_frame;
			}

			// Shift morph keyframes
			for (auto& frame : file_data.vmd->face_frames)
			{
				frame.frame -= start_frame;
			}

			// Shift IK keyframes
			for (auto& frame : file_data.vmd->ik_frames)
			{
				frame.frame -= start_frame;
			}
		}

		const char* filepath = ExpGetPmdFilenameUtf8(i);
		std::wstring filepath_wstring = umbase::UMStringUtil::utf16_to_wstring(umbase::UMStringUtil::utf8_to_utf16(filepath));
		std::wstring filename_wstring = PathFindFileNameW(filepath_wstring.c_str());
		if (filename_wstring.empty())
		{
			std::wstring error_message = L"Unable to get pmd/pmx filepath.";
			::MessageBoxW(NULL, error_message.c_str(), L"Error", MB_OK | MB_SETFOREGROUND | MB_ICONERROR);
			continue;
		}

		wchar_t filename_buffer[MAX_PATH];
		wcscpy_s(filename_buffer, MAX_PATH, filename_wstring.c_str());
		PathRenameExtensionW(filename_buffer, L".vmd");
		std::wstring base_output_filename(filename_buffer);

		// Resolves filename collisions that occur when the same model is imported multiple times.
		int count = output_name_counts[base_output_filename]++;
		std::wstring final_output_filename = base_output_filename;
		if (count > 0)
		{
			wchar_t name_buffer[MAX_PATH];
			wcscpy_s(name_buffer, MAX_PATH, base_output_filename.c_str());
			PathRemoveExtensionW(name_buffer);
			std::wstring name_without_ext(name_buffer);
			final_output_filename = name_without_ext + L" (" + std::to_wstring(count + 1) + L").vmd";
		}

		wchar_t pathBuffer[MAX_PATH];
		PathCombineW(pathBuffer, archive.output_path.c_str(), final_output_filename.c_str());
		std::wstring output_filepath = pathBuffer;
		file_data.vmd->SaveToFile(output_filepath);
	}

	VMDArchive::instance().end();
	return true;
}

static bool end_gltf_export() {
	VMDArchive &archive = VMDArchive::instance();
	if (!archive.is_start_vmd_export_called) {
		archive.end();
		ShowFrameRangeConfigError();
		return false;
	}

	if (archive.has_bone_name_error) {
		archive.end();
		ShowInvalidBoneNameError();
		return false;
	}

	const int pmd_num = ExpGetPmdNum();
	std::map<std::wstring, int> output_name_counts;
	bool success = true;
	for (int i = 0; i < pmd_num; ++i) {
		FileDataForVMD &file_data = archive.data_list.at(i);
		if (!file_data.pmd && !file_data.pmx) {
			continue;
		}

		const char *filepath = ExpGetPmdFilenameUtf8(i);
		std::wstring filepath_wstring = umbase::UMStringUtil::utf16_to_wstring(
			umbase::UMStringUtil::utf8_to_utf16(filepath));
		std::wstring filename_wstring = PathFindFileNameW(filepath_wstring.c_str());
		if (filename_wstring.empty()) {
			::MessageBoxW(NULL, L"Unable to get pmd/pmx filepath.", L"Error",
			              MB_OK | MB_SETFOREGROUND | MB_ICONERROR);
			success = false;
			continue;
		}

		wchar_t filename_buffer[MAX_PATH];
		wcscpy_s(filename_buffer, MAX_PATH, filename_wstring.c_str());
		PathRenameExtensionW(filename_buffer, L".gltf");
		std::wstring base_output_filename(filename_buffer);

		const int count = output_name_counts[base_output_filename]++;
		std::wstring final_output_filename = base_output_filename;
		if (count > 0) {
			wchar_t name_buffer[MAX_PATH];
			wcscpy_s(name_buffer, MAX_PATH, base_output_filename.c_str());
			PathRemoveExtensionW(name_buffer);
			std::wstring name_without_ext(name_buffer);
			final_output_filename = name_without_ext + L" (" + std::to_wstring(count + 1) + L").gltf";
		}

		wchar_t bin_filename_buffer[MAX_PATH];
		wcscpy_s(bin_filename_buffer, MAX_PATH, final_output_filename.c_str());
		PathRenameExtensionW(bin_filename_buffer, L".bin");

		wchar_t gltf_path_buffer[MAX_PATH];
		wchar_t bin_path_buffer[MAX_PATH];
		if (!PathCombineW(gltf_path_buffer, archive.output_path.c_str(), final_output_filename.c_str()) ||
		    !PathCombineW(bin_path_buffer, archive.output_path.c_str(), bin_filename_buffer)) {
			success = false;
			continue;
		}
		const bool file_success = write_gltf_file(
			file_data, std::wstring(gltf_path_buffer), std::wstring(bin_path_buffer));
		success = file_success && success;
	}

	archive.end();
	return success;
}

static void init_file_data(FileDataForVMD& data)
{
	if (data.pmd)
	{
		const std::vector<pmd::PmdBone>& bones = data.pmd->bones;
		const std::vector<pmd::PmdRigidBody>& rigids = data.pmd->rigid_bodies;
		std::map<int, int> bone_to_rigid_map;
		for (size_t i = 0, isize = bones.size(); i < isize; ++i)
		{
			const pmd::PmdBone& bone = bones[i];
			const uint16_t parent_bone = bone.parent_bone_index;
			data.parent_index_map[i] = (parent_bone == 0xFFFF) ? -1 : parent_bone;
			data.bone_name_map[i] = bone.name;
			if (bone.bone_type == pmd::BoneType::IkEffectable)
			{
				data.ik_bone_map[i] = 1;
			}
			if (bone.bone_type == pmd::BoneType::IkEffector)
			{
				data.ik_frame_bone_map[i] = 1;
			}
			// fuyo
			if (bone.bone_type == pmd::BoneType::RotationEffectable) // Type 5
			{
				// For Type 5, the ik_parent_bone_index field stores the grant parent's index
				const int grant_parent_index = (bone.ik_parent_bone_index == 0xFFFF) ? -1 : bone.ik_parent_bone_index;
				if (grant_parent_index >= 0)
				{
					data.fuyo_target_map[grant_parent_index] = 1;
					data.fuyo_bone_map[i] = 1;
				}
			}
			else if (bone.bone_type == pmd::BoneType::RotationMovement) // Type 9
			{
				// For Type 9, the tail_pos_bone_index field stores the grant parent's index
				const int grant_parent_index = (bone.tail_pos_bone_index == 0xFFFF) ? -1 : bone.tail_pos_bone_index;
				if (grant_parent_index >= 0)
				{
					data.fuyo_target_map[grant_parent_index] = 1;
					data.fuyo_bone_map[i] = 1;
				}
			}
		}
		// morph (face)
		const std::vector<pmd::PmdFace>& faces = data.pmd->faces;
		for (size_t i = 0, isize = faces.size(); i < isize; ++i)
		{
			const pmd::PmdFace& face = faces[i];
			data.morph_name_map[i] = face.name;
		}

		for (size_t i = 0, isize = rigids.size(); i < isize; ++i)
		{
			const pmd::PmdRigidBody& rigid = rigids[i];
			const uint16_t related_bone = rigid.related_bone_index;
			const int target_bone = (related_bone == 0xFFFF) ? -1 : related_bone;
			if (target_bone < 0)
			{
				continue;
			}
			bone_to_rigid_map[target_bone] = i;
			if (rigid.rigid_type != pmd::RigidBodyType::BoneConnected)
			{
				if (data.bone_name_map.find(target_bone) != data.bone_name_map.end())
				{
					if (rigid.rigid_type == pmd::RigidBodyType::ConnectedPhysics)
					{
						data.physics_bone_map[target_bone] = 2;
					}
					else
					{
						data.physics_bone_map[target_bone] = 1;
					}
				}
			}
		}
		// expect for rigid_type == BoneConnected
		{
			std::vector<int> parent_physics_bone_list;
			for (const auto& rigid : rigids)
			{
				const uint16_t related_bone = rigid.related_bone_index;
				const int target_bone = (related_bone == 0xFFFF) ? -1 : related_bone;
				if (target_bone < 0) // Avoid dangerous map[-1] access, which auto-creates a buggy {-1, 0} mapping.
				{
					continue;
				}
				const int parent_bone = data.parent_index_map[target_bone];
				if (parent_bone < 0)
				{
					continue;
				}
				if (data.physics_bone_map.find(target_bone) != data.physics_bone_map.end() &&
					bone_to_rigid_map.find(parent_bone) != bone_to_rigid_map.end())
				{
					parent_physics_bone_list.push_back(parent_bone);
				}
			}
			for (int parent_bone : parent_physics_bone_list)
			{
				const pmd::PmdRigidBody& parent_rigid = rigids[bone_to_rigid_map[parent_bone]];
				if (parent_rigid.rigid_type == pmd::RigidBodyType::BoneConnected)
				{
					data.physics_bone_map[parent_bone] = 0;
				}
			}
		}
	}
	else if (data.pmx)
	{
		const int bone_count = static_cast<int>(data.pmx->bones.size());
		for (int i = 0; i < bone_count; ++i)
		{
			const pmx::PmxBone& bone = data.pmx->bones[i];
			const int parent_bone = bone.parent_index;
			data.parent_index_map[i] = parent_bone;
			oguna::EncodingConverter::Utf16ToCp932(bone.bone_name.c_str(), static_cast<int>(bone.bone_name.length()), &data.bone_name_map[i]);
			for (int k = 0; k < bone.ik_link_count; ++k)
			{
				const pmx::PmxIkLink& link = bone.ik_links[k];
				if (link.link_target < 0)
				{
					continue;
				}
				data.ik_bone_map[link.link_target] = 1;
				data.ik_frame_bone_map[i] = 1;
			}
			if ((bone.bone_flag & 0x0100) || (bone.bone_flag & 0x0200))
			{
				if (bone.grant_parent_index < 0)
				{
					continue;
				}
				data.fuyo_target_map[bone.grant_parent_index] = 1;
				data.fuyo_bone_map[i] = 1;
			}
		}

		const int rigid_count = static_cast<int>(data.pmx->rigid_bodies.size());
		std::map<int, int> bone_to_rigid_map;
		for (int i = 0; i < rigid_count; ++i)
		{
			const pmx::PmxRigidBody& rigid = data.pmx->rigid_bodies[i];
			const int target_bone = rigid.target_bone;
			if (target_bone < 0)
			{
				continue;
			}
			bone_to_rigid_map[target_bone] = i;
			if (rigid.physics_calc_type != 0)
			{
				if (data.bone_name_map.find(target_bone) != data.bone_name_map.end())
				{
					if (rigid.physics_calc_type == 2)
					{
						data.physics_bone_map[target_bone] = 2;
					}
					else
					{
						data.physics_bone_map[target_bone] = 1;
					}
				}
			}
		}
		// expect for physics_calc_type == 0
		{
			std::vector<int> parent_physics_bone_list;
			for (int i = 0; i < rigid_count; ++i)
			{
				const pmx::PmxRigidBody& rigid = data.pmx->rigid_bodies[i];
				const int target_bone = rigid.target_bone;
				if (target_bone < 0) // Avoid dangerous map[-1] access, which auto-creates a buggy {-1, 0} mapping.
				{
					continue;
				}
				const int parent_bone = data.parent_index_map[target_bone];
				if (parent_bone < 0)
				{
					continue;
				}
				if (data.physics_bone_map.find(target_bone) != data.physics_bone_map.end() &&
					bone_to_rigid_map.find(parent_bone) != bone_to_rigid_map.end())
				{
					parent_physics_bone_list.push_back(parent_bone);
				}
			}
			for (int parent_bone : parent_physics_bone_list)
			{
				const pmx::PmxRigidBody& parent_rigid = data.pmx->rigid_bodies[bone_to_rigid_map[parent_bone]];
				if (parent_rigid.physics_calc_type == 0)
				{
					data.physics_bone_map[parent_bone] = 0;
				}
			}
		}
		// morph (face)
		const int morph_count = static_cast<int>(data.pmx->morphs.size());
		for (int i = 0; i < morph_count; ++i)
		{
			const pmx::PmxMorph& morph = data.pmx->morphs[i];
			std::string morph_name;
			oguna::EncodingConverter::Utf16ToCp932(morph.morph_name.c_str(), static_cast<int>(morph.morph_name.length()), &morph_name);
			data.morph_name_map[i] = morph_name;
		}
	}
}

static Imath::Matrix44<double> to_imath_matrix(const D3DMATRIX& mat)
{
	return Imath::Matrix44<double>(
		mat.m[0][0], mat.m[0][1], mat.m[0][2], mat.m[0][3],
		mat.m[1][0], mat.m[1][1], mat.m[1][2], mat.m[1][3],
		mat.m[2][0], mat.m[2][1], mat.m[2][2], mat.m[2][3],
		mat.m[3][0], mat.m[3][1], mat.m[3][2], mat.m[3][3]);
}

// Helper function: Calculate VMD bone frame based on bone index
static vmd::VmdBoneFrame calculate_bone_frame(
	int model_index,				// Model index (i)
	int bone_index,					// Bone index (k)
	int current_frame,				// Current frame number
	const FileDataForVMD& file_data // File data
)
{
	vmd::VmdBoneFrame bone_frame;
	bone_frame.frame = current_frame;
	bone_frame.name = ExpGetPmdBoneName(model_index, bone_index);

	// Get bone name for validation
	const char* bone_name = ExpGetPmdBoneName(model_index, bone_index);

	// Validate bone name mapping
	auto it = file_data.bone_name_map.find(bone_index);
	if (it == file_data.bone_name_map.end() || it->second != bone_name)
	{
		// If validation fails, return default bone_frame
		return bone_frame;
	}

	// Get initial position
	Imath::Vec3<float> initial_trans;
	if (file_data.pmd)
	{
		const pmd::PmdBone& bone = file_data.pmd->bones[bone_index];
		if (bone.bone_type == pmd::BoneType::Invisible)
		{
			return bone_frame; // Return default values
		}
		initial_trans.x = bone.bone_head_pos[0];
		initial_trans.y = bone.bone_head_pos[1];
		initial_trans.z = bone.bone_head_pos[2];
	}
	else if (file_data.pmx)
	{
		const pmx::PmxBone& bone = file_data.pmx->bones[bone_index];
		initial_trans.x = bone.position[0];
		initial_trans.y = bone.position[1];
		initial_trans.z = bone.position[2];
	}

	// Get transformation matrices
	Imath::Matrix44<double> world = to_imath_matrix(ExpGetPmdBoneWorldMat(model_index, bone_index));
	Imath::Matrix44<double> local = world;
	int parent_index = file_data.parent_index_map.count(bone_index) ? file_data.parent_index_map.at(bone_index) : -1;
	if (parent_index >= 0)
	{
		Imath::Matrix44<double> parent_world = to_imath_matrix(ExpGetPmdBoneWorldMat(model_index, parent_index));
		local = world * parent_world.inverse();
	}

	// Calculate VMD position
	// Step 1: Subtract own initial position
	bone_frame.position[0] = static_cast<float>(local[3][0]) - initial_trans.x;
	bone_frame.position[1] = static_cast<float>(local[3][1]) - initial_trans.y;
	bone_frame.position[2] = static_cast<float>(local[3][2]) - initial_trans.z;
	// Step 2: Add parent bone's initial position
	if (parent_index >= 0)
	{
		if (file_data.pmd && parent_index < static_cast<int>(file_data.pmd->bones.size()))
		{
			const pmd::PmdBone& parent_bone = file_data.pmd->bones[parent_index];
			bone_frame.position[0] += parent_bone.bone_head_pos[0];
			bone_frame.position[1] += parent_bone.bone_head_pos[1];
			bone_frame.position[2] += parent_bone.bone_head_pos[2];
		}
		else if (file_data.pmx && parent_index < static_cast<int>(file_data.pmx->bones.size()))
		{
			const pmx::PmxBone& parent_bone = file_data.pmx->bones[parent_index];
			bone_frame.position[0] += parent_bone.position[0];
			bone_frame.position[1] += parent_bone.position[1];
			bone_frame.position[2] += parent_bone.position[2];
		}
	}

	// Calculate rotation
	Imath::Matrix44<double> rotation_matrix = local;
	rotation_matrix[3][0] = rotation_matrix[3][1] = rotation_matrix[3][2] = 0.0;
	Imath::Quat<double> quat = Imath::extractQuat(rotation_matrix);

	// Normalize result
	quat.normalize();

	bone_frame.orientation[0] = static_cast<float>(quat.v.x);
	bone_frame.orientation[1] = static_cast<float>(quat.v.y);
	bone_frame.orientation[2] = static_cast<float>(quat.v.z);
	bone_frame.orientation[3] = static_cast<float>(quat.r);

	return bone_frame;
}

// Helper function: Calculate VMD face frame based on morph index
static vmd::VmdFaceFrame calculate_face_frame(
	int model_index,				// Model index (i)
	int morph_index,				// Morph index (m)
	int current_frame,				// Current frame number
	const FileDataForVMD& file_data // File data
)
{
	vmd::VmdFaceFrame face_frame;
	face_frame.frame = static_cast<uint32_t>(current_frame);

	// Get morph name
	const char* morph_name = ExpGetPmdMorphName(model_index, morph_index);
	face_frame.face_name = morph_name;

	// Validate morph name mapping
	auto it = file_data.morph_name_map.find(morph_index);
	if (it == file_data.morph_name_map.end() || it->second != morph_name)
	{
		// If validation fails, return default face_frame with 0 weight
		face_frame.weight = 0.0f;
		return face_frame;
	}

	// Get morph value from MMD
	face_frame.weight = ExpGetPmdMorphValue(model_index, morph_index);

	return face_frame;
}

static bool execute_vmd_export(const int currentframe)
{
	VMDArchive& archive = VMDArchive::instance();

	if (!archive.is_start_vmd_export_called)
	{
		if (!archive.is_start_vmd_export_warning_shown)
		{
			archive.is_start_vmd_export_warning_shown = true;
			ShowFrameRangeConfigError();
		}
		return false;
	}

	const BridgeParameter& parameter = BridgeParameter::instance();
	const int pmd_num = ExpGetPmdNum();

	if (currentframe == parameter.start_frame)
	{
		for (int i = 0; i < pmd_num; ++i)
		{
			FileDataForVMD& file_data = archive.data_list.at(i);
			init_file_data(file_data);

			file_data.vmd = std::make_unique<vmd::VmdMotion>();
			if (file_data.pmd)
			{
				file_data.vmd->model_name = file_data.pmd->header.name;
			}
			else if (file_data.pmx)
			{
				oguna::EncodingConverter::Utf16ToCp932(file_data.pmx->model_name.c_str(), static_cast<int>(file_data.pmx->model_name.length()), &file_data.vmd->model_name);
			}
		}
	}

	for (int i = 0; i < pmd_num; ++i)
	{
		FileDataForVMD& file_data = archive.data_list.at(i);
		const int bone_num = ExpGetPmdBoneNum(i);
		for (int k = 0; k < bone_num; ++k)
		{
			const char* bone_name = ExpGetPmdBoneName(i, k);

			if (!archive.has_bone_name_error && (bone_name == nullptr || strlen(bone_name) == 0))
			{
				archive.has_bone_name_error = true;
				ShowInvalidBoneNameError();
			}

			// Validate bone name mapping
			{
				auto it = file_data.bone_name_map.find(k);
				if (it == file_data.bone_name_map.end() || it->second != bone_name)
				{
					continue;
				}
			}

			// Export mode filtering
			const bool is_ik_effector_bone = file_data.ik_frame_bone_map.count(k) > 0;
			// const bool is_affected_by_ik = file_data.ik_bone_map.count(k) > 0;
			// const bool is_fuyo_effector_bone = file_data.fuyo_target_map.find(k) != file_data.fuyo_target_map.end();
			// const bool is_affected_by_fuyo = file_data.fuyo_bone_map.count(k) > 0;
			bool is_physics_bone = false;
			bool is_simulated_physics_bone = false;
			bool is_non_simulated_physics_bone = false;
			{
				auto it = file_data.physics_bone_map.find(k);
				is_physics_bone = (it != file_data.physics_bone_map.end());
				if (is_physics_bone)
				{
					if (it->second == 0) // ボーン追従
					{
						is_non_simulated_physics_bone = true;
					}
					else
					{
						is_simulated_physics_bone = true;
					}
				}
			}

			// Since IK is baked to FK, skip exporting IK bone motion keyframes
			if (is_ik_effector_bone)
			{
				if (!archive.export_ik_bone_animation)
				{
					continue;
				}
			} else { // This is an FK bone
				if (is_simulated_physics_bone) {
					if (archive.export_fk_bone_animation_mode < 0) {
						continue;
					}
				} else if (archive.export_fk_bone_animation_mode == 0) { // 0: Simulated physics bones only
					continue;
				}
			}

			// Use helper function to calculate bone frame
			vmd::VmdBoneFrame bone_frame = calculate_bone_frame(i, k, currentframe, file_data);

			// -1: Only FK bones.
			// 1: All FK bones. Exclude 付与親 and Bone Morph influences from bone animation. (For MMD / MMD Tools, which re-apply them at runtime)
			if (std::abs(archive.export_fk_bone_animation_mode) == 1) {
				// Remove grant parent influence
				if (file_data.pmx && k < static_cast<int>(file_data.pmx->bones.size()))
				{
					const pmx::PmxBone& current_bone = file_data.pmx->bones[k];
					const uint16_t grant_flags = current_bone.bone_flag & 0x0300; // 0x0100 | 0x0200

					if (grant_flags && current_bone.grant_parent_index >= 0 &&
						current_bone.grant_parent_index < static_cast<int>(file_data.pmx->bones.size()))
					{
						// Calculate grant parent bone frame
						vmd::VmdBoneFrame grant_parent_frame = calculate_bone_frame(
							i, current_bone.grant_parent_index, currentframe, file_data);

						const float grant_weight = current_bone.grant_weight;

						// Remove position grant influence
						if (grant_flags & 0x0200) // Position grant
						{
							bone_frame.position[0] -= grant_parent_frame.position[0] * grant_weight;
							bone_frame.position[1] -= grant_parent_frame.position[1] * grant_weight;
							bone_frame.position[2] -= grant_parent_frame.position[2] * grant_weight;
						}

						// Remove rotation grant influence
						if (grant_flags & 0x0100) // Rotation grant
						{
							// Create Imath quaternion objects
							Imath::Quatf current_quat(bone_frame.orientation[3],
													  bone_frame.orientation[0],
													  bone_frame.orientation[1],
													  bone_frame.orientation[2]);
							Imath::Quatf parent_quat(grant_parent_frame.orientation[3],
													 grant_parent_frame.orientation[0],
													 grant_parent_frame.orientation[1],
													 grant_parent_frame.orientation[2]);

							// Create identity quaternion for interpolation
							Imath::Quatf identity = Imath::Quatf::identity();

							// Use slerp to calculate scaled parent quaternion
							// scaled_parent_quat = slerp(identity, parent_quat, grant_weight)
							Imath::Quatf scaled_parent_quat = slerpShortestArc(identity, parent_quat, grant_weight);

							// Remove grant influence: current_pure = current * scaled_parent_quat^(-1)
							Imath::Quatf pure_quat = current_quat * scaled_parent_quat.inverse();

							// Normalize result
							pure_quat.normalize();

							// Convert back to VMD format (x, y, z, w)
							bone_frame.orientation[0] = pure_quat.v.x;
							bone_frame.orientation[1] = pure_quat.v.y;
							bone_frame.orientation[2] = pure_quat.v.z;
							bone_frame.orientation[3] = pure_quat.r;
						}
					}
				}
				else if (file_data.pmd && k < static_cast<int>(file_data.pmd->bones.size()))
				{
					const pmd::PmdBone& current_bone = file_data.pmd->bones[k];
					int grant_parent_index = -1;
					float grant_weight = 0.0f;
					bool grant_rotation = false;
					// pmd grant parent only supports rotation (RotationEffectable and RotationMovement)

					if (current_bone.bone_type == pmd::BoneType::RotationEffectable) // Type 5
					{
						grant_parent_index = (current_bone.ik_parent_bone_index == 0xFFFF) ? -1 : current_bone.ik_parent_bone_index;
						grant_weight = 1.0f;
						grant_rotation = true;
					}
					else if (current_bone.bone_type == pmd::BoneType::RotationMovement) // Type 9
					{
						grant_parent_index = (current_bone.tail_pos_bone_index == 0xFFFF) ? -1 : current_bone.tail_pos_bone_index;
						// For Type 9, the ik_parent_bone_index field stores the weight
						grant_weight = static_cast<float>(current_bone.ik_parent_bone_index) / 100.0f;
						grant_rotation = true;
					}

					if (grant_rotation && grant_parent_index >= 0 &&
						grant_parent_index < static_cast<int>(file_data.pmd->bones.size()))
					{
						// Calculate grant parent bone frame
						vmd::VmdBoneFrame grant_parent_frame = calculate_bone_frame(i, grant_parent_index, currentframe, file_data);

						// Remove the influence of rotation grant
						Imath::Quatf current_quat(bone_frame.orientation[3],
												  bone_frame.orientation[0],
												  bone_frame.orientation[1],
												  bone_frame.orientation[2]);
						Imath::Quatf parent_quat(grant_parent_frame.orientation[3],
												 grant_parent_frame.orientation[0],
												 grant_parent_frame.orientation[1],
												 grant_parent_frame.orientation[2]);

						Imath::Quatf identity = Imath::Quatf::identity();
						Imath::Quatf scaled_parent_quat = slerpShortestArc(identity, parent_quat, grant_weight);
						Imath::Quatf pure_quat = current_quat * scaled_parent_quat.inverse();
						pure_quat.normalize();

						bone_frame.orientation[0] = pure_quat.v.x;
						bone_frame.orientation[1] = pure_quat.v.y;
						bone_frame.orientation[2] = pure_quat.v.z;
						bone_frame.orientation[3] = pure_quat.r;
					}
				}

				// Remove bone morph influence (pmd doesn't have bone morph)
				if (file_data.pmx)
				{
					Imath::Vec3 total_pos_offset(0.0f, 0.0f, 0.0f);
					Imath::Quatf total_rot_offset = Imath::Quatf::identity();
					const int morph_num = ExpGetPmdMorphNum(i);

					for (int m = 0; m < morph_num; ++m)
					{
						const float morph_weight = ExpGetPmdMorphValue(i, m);
						if (morph_weight == 0.0f)
						{
							continue;
						}

						// Validate morph name mapping (safety check)
						const char* morph_name = ExpGetPmdMorphName(i, m);
						auto it = file_data.morph_name_map.find(m);
						if (it == file_data.morph_name_map.end() || it->second != morph_name)
						{
							continue;
						}

						const pmx::PmxMorph& morph = file_data.pmx->morphs[m];

						// Case 1: Direct Bone Morph
						if (morph.morph_type == pmx::MorphType::Bone)
						{
							for (const auto& bone_offset : morph.bone_offsets)
							{
								if (bone_offset.bone_index == k)
								{
									// Accumulate position offset
									total_pos_offset.x += bone_offset.translation[0] * morph_weight;
									total_pos_offset.y += bone_offset.translation[1] * morph_weight;
									total_pos_offset.z += bone_offset.translation[2] * morph_weight;

									// Accumulate rotation offset
									Imath::Quatf offset_quat(bone_offset.rotation[3],  // w
															 bone_offset.rotation[0],  // x
															 bone_offset.rotation[1],  // y
															 bone_offset.rotation[2]); // z
									Imath::Quatf slerped_rot = slerpShortestArc(Imath::Quatf::identity(), offset_quat, morph_weight);
									total_rot_offset = slerped_rot * total_rot_offset; // Apply in order
								}
							}
						}
						// Case 2: Group Morph (which might contain Bone Morphs)
						else if (morph.morph_type == pmx::MorphType::Group)
						{
							for (const auto& group_offset : morph.group_offsets)
							{
								const float effective_weight = morph_weight * group_offset.morph_weight;
								if (effective_weight == 0.0f ||
									group_offset.morph_index < 0 ||
									group_offset.morph_index >= file_data.pmx->morphs.size())
								{
									continue;
								}

								const pmx::PmxMorph& child_morph = file_data.pmx->morphs[group_offset.morph_index];
								if (child_morph.morph_type == pmx::MorphType::Bone)
								{
									for (const auto& bone_offset : child_morph.bone_offsets)
									{
										if (bone_offset.bone_index == k)
										{
											// Accumulate position offset
											total_pos_offset.x += bone_offset.translation[0] * effective_weight;
											total_pos_offset.y += bone_offset.translation[1] * effective_weight;
											total_pos_offset.z += bone_offset.translation[2] * effective_weight;

											// Accumulate rotation offset
											Imath::Quatf offset_quat(bone_offset.rotation[3],  // w
																	 bone_offset.rotation[0],  // x
																	 bone_offset.rotation[1],  // y
																	 bone_offset.rotation[2]); // z
											Imath::Quatf slerped_rot = slerpShortestArc(Imath::Quatf::identity(), offset_quat, effective_weight);
											total_rot_offset = slerped_rot * total_rot_offset; // Apply in order
										}
									}
								}
							}
						}
					}

					// If any morph affected this bone, apply the total inverse transform
					if (total_pos_offset.length2() > 1e-6f || std::abs(total_rot_offset.r - 1.0f) > 1e-6f)
					{
						// Remove position influence
						bone_frame.position[0] -= total_pos_offset.x;
						bone_frame.position[1] -= total_pos_offset.y;
						bone_frame.position[2] -= total_pos_offset.z;

						// Remove rotation influence
						Imath::Quatf current_quat(bone_frame.orientation[3],  // w
												  bone_frame.orientation[0],  // x
												  bone_frame.orientation[1],  // y
												  bone_frame.orientation[2]); // z

						Imath::Quatf pure_quat = current_quat * total_rot_offset.inverse();
						pure_quat.normalize();

						bone_frame.orientation[0] = pure_quat.v.x;
						bone_frame.orientation[1] = pure_quat.v.y;
						bone_frame.orientation[2] = pure_quat.v.z;
						bone_frame.orientation[3] = pure_quat.r;
					}
				}
			}
			else // 2: All FK bones. Bake all influences (付与親, Bone Morph) into bone animation. (For other 3D software)
			{
				// Keep grant parent and bone morph influence
				// The FK animation is already baked, do nothing
			}

			// Simplify animations
			if (file_data.last_bone_frame.find(k) == file_data.last_bone_frame.end()) {
				file_data.vmd->bone_frames.push_back(bone_frame);
				file_data.last_bone_frame[k] = bone_frame;
			} else {
				vmd::VmdBoneFrame& last_frame = file_data.last_bone_frame[k];
				if (std::pow(bone_frame.position[0] - last_frame.position[0], 2) +
						std::pow(bone_frame.position[1] - last_frame.position[1], 2) +
						std::pow(bone_frame.position[2] - last_frame.position[2], 2) > 1e-8f ||
					std::abs(bone_frame.orientation[0] * last_frame.orientation[0] +
							 bone_frame.orientation[1] * last_frame.orientation[1] +
							 bone_frame.orientation[2] * last_frame.orientation[2] +
							 bone_frame.orientation[3] * last_frame.orientation[3]) < 1 - 1e-5f) {
								file_data.vmd->bone_frames.push_back(bone_frame);
								file_data.last_bone_frame[k] = bone_frame;
				}
			}
		}

		// Handle IK frames for the first frame
		if (archive.add_turn_off_ik_keyframe && currentframe == parameter.start_frame)
		{
			vmd::VmdIkFrame ik_frame;
			ik_frame.frame = currentframe;
			ik_frame.display = true;
			for (auto it = file_data.ik_frame_bone_map.begin(); it != file_data.ik_frame_bone_map.end(); ++it)
			{
				if (file_data.bone_name_map.find(it->first) != file_data.bone_name_map.end())
				{
					vmd::VmdIkEnable ik_enable;
					ik_enable.ik_name = file_data.bone_name_map[it->first];
					ik_enable.enable = false;
					ik_frame.ik_enable.push_back(ik_enable);
				}
			}
			file_data.vmd->ik_frames.push_back(ik_frame);
		}

		// morph (face)
		if (archive.export_morph_animation)
		{
			const int morph_num = ExpGetPmdMorphNum(i);
			if (archive.export_vertex_morph_animation_only && file_data.pmx)
			{
				// Bake all vertex morph weights, including contributions from group morphs.
				// Key: vertex morph index, Value: accumulated weight
				std::map<int, float> vertex_morph_weights;

				for (int m = 0; m < morph_num; ++m)
				{
					// Validate morph name mapping
					const char* morph_name = ExpGetPmdMorphName(i, m);
					auto it = file_data.morph_name_map.find(m);
					if (it == file_data.morph_name_map.end() || it->second != morph_name)
					{
						continue;
					}

					const pmx::PmxMorph& morph = file_data.pmx->morphs[m];
					const float morph_weight = ExpGetPmdMorphValue(i, m);

					if (morph.morph_type == pmx::MorphType::Vertex)
					{
						// Accumulate direct vertex morph contribution
						vertex_morph_weights[m] += morph_weight;
					}
					else if (morph.morph_type == pmx::MorphType::Group)
					{
						// Expand group morph: accumulate each vertex morph child
						for (const auto& group_offset : morph.group_offsets)
						{
							const int child_index = group_offset.morph_index;
							if (child_index < 0 || child_index >= morph_num)
							{
								continue;
							}

							const pmx::PmxMorph& child_morph = file_data.pmx->morphs[child_index];
							if (child_morph.morph_type != pmx::MorphType::Vertex)
							{
								continue;
							}

							// Validate child morph name mapping
							const char* child_morph_name = ExpGetPmdMorphName(i, child_index);
							auto child_it = file_data.morph_name_map.find(child_index);
							if (child_it == file_data.morph_name_map.end() || child_it->second != child_morph_name)
							{
								continue;
							}

							const float effective_weight = morph_weight * group_offset.morph_weight;
							vertex_morph_weights[child_index] += effective_weight;
						}
					}
				}

				// Emit one face frame per vertex morph with the accumulated weight
				for (const auto& [morph_index, accumulated_weight] : vertex_morph_weights)
				{
					vmd::VmdFaceFrame face_frame;
					face_frame.frame = static_cast<uint32_t>(currentframe);
					face_frame.face_name = file_data.morph_name_map.at(morph_index);
					face_frame.weight = accumulated_weight;
					file_data.vmd->face_frames.push_back(face_frame);
				}
			}
			else
			{
				// Default path: export all morphs as-is
				for (int m = 0; m < morph_num; ++m)
				{
					const char* morph_name = ExpGetPmdMorphName(i, m);

					// Validate morph name mapping
					auto it = file_data.morph_name_map.find(m);
					if (it == file_data.morph_name_map.end() || it->second != morph_name)
					{
						continue;
					}

					// Use helper function to calculate face frame
					vmd::VmdFaceFrame face_frame = calculate_face_frame(i, m, currentframe, file_data);
					file_data.vmd->face_frames.push_back(face_frame);
				}
			}
		}
	}

	return true;
}

// ---------------------------------------------------------------------------
PYBIND11_MODULE(mmdbridge_vmd, m)
{
	m.doc() = "MMD Bridge VMD export module";
	m.def("start_vmd_export", start_vmd_export,
		  py::arg("export_fk_bone_animation_mode"),
		  py::arg("export_ik_bone_animation"),
		  py::arg("add_turn_off_ik_keyframe"),
		  py::arg("export_morph_animation"),
		  py::arg("export_vertex_morph_animation_only"));
	m.def("end_vmd_export", end_vmd_export);
	m.def("end_gltf_export", end_gltf_export);
	m.def("execute_vmd_export", execute_vmd_export);
}

#endif // WITH_VMD

// ---------------------------------------------------------------------------
// clang-format off
#ifdef WITH_VMD
	void InitVMD()
	{
		PyImport_AppendInittab("mmdbridge_vmd", PyInit_mmdbridge_vmd);
	}
	void DisposeVMD()
	{
		VMDArchive::instance().end();
	}
#else
	void InitVMD() {}
	void DisposeVMD() {}
#endif // WITH_VMD
// clang-format on
