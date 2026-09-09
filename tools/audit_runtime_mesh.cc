// Read the exact geometry consumed by Gazebo, not just the source XML.
#include <gz/common/Material.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

void vectorJson(const gz::math::Vector3d &v) {
  std::cout << '[' << v.X() << ',' << v.Y() << ',' << v.Z() << ']';
}

int main(int argc, char **argv) {
  if (argc < 4 || std::string(argv[1]) != "--dump") {
    std::cerr << "Usage: audit_runtime_mesh --dump DIR MESH [MESH...]\n";
    return 2;
  }
  const std::filesystem::path dumpDir(argv[2]);
  std::filesystem::create_directories(dumpDir);
  std::cout << std::setprecision(17) << "{\"meshes\":[";
  for (int i = 3; i < argc; ++i) {
    if (i != 3) std::cout << ',';
    const auto *mesh = gz::common::MeshManager::Instance()->Load(argv[i]);
    if (!mesh) return 1;
    std::cout << "{\"path\":" << std::quoted(argv[i]) << ",\"submeshes\":[";
    for (unsigned j = 0; j < mesh->SubMeshCount(); ++j) {
      if (j) std::cout << ',';
      const auto sub = mesh->SubMeshByIndex(j).lock();
      const auto filename = std::to_string(i - 3) + "_" + std::to_string(j) + ".bin";
      std::ofstream data(dumpDir / filename, std::ios::binary);
      for (unsigned k = 0; k < sub->IndexCount(); ++k) {
        const auto v = sub->Vertex(sub->Index(k));
        const double xyz[] = {v.X(), v.Y(), v.Z()};
        data.write(reinterpret_cast<const char *>(xyz), sizeof(xyz));
      }
      if (!data) return 1;
      std::cout << "{\"name\":" << std::quoted(sub->Name())
                << ",\"triangle_count\":" << sub->IndexCount() / 3
                << ",\"normal_count\":" << sub->NormalCount()
                << ",\"vertex_count\":" << sub->VertexCount()
                << ",\"dump\":" << std::quoted(filename) << ",\"min_m\":";
      vectorJson(sub->Min());
      std::cout << ",\"max_m\":";
      vectorJson(sub->Max());
      const auto index = sub->GetMaterialIndex();
      if (index) {
        const auto material = mesh->MaterialByIndex(*index);
        const auto color = material->Diffuse();
        std::cout << ",\"diffuse\":[" << color.R() << ',' << color.G()
                  << ',' << color.B() << ',' << color.A() << ']';
      }
      std::cout << '}';
    }
    std::cout << "]}";
  }
  std::cout << "]}\n";
}
