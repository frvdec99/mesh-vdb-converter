#include <iostream>
#include <vector>
#include <chrono>

#include "openvdb/openvdb.h"
#include "openvdb/tools/MeshToVolume.h"
#include "openvdb/tools/Dense.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

struct Mesh {
    std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec3I> faces;
    openvdb::BBoxd bbox;
};

void read_obj(const std::string& filepath, Mesh &mesh) {
    mesh.points.clear();
    mesh.faces.clear();

    tinyobj::ObjReaderConfig reader_config;
    reader_config.triangulate = true;
    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile(filepath, reader_config)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
        }
        exit(1);
    }
    if (!reader.Warning().empty()) std::cout << "TinyObjReader: " << reader.Warning();

    auto &attrib = reader.GetAttrib();
    auto &shapes = reader.GetShapes();

    for (size_t i = 0; i < attrib.vertices.size(); i += 3) {
        tinyobj::real_t vx = attrib.vertices[i + 0];
        tinyobj::real_t vy = attrib.vertices[i + 1];
        tinyobj::real_t vz = attrib.vertices[i + 2];
        openvdb::Vec3s v = openvdb::Vec3s(vx, vz, vy);
        mesh.bbox.expand(v);
        mesh.points.push_back(v);
    }

    for (const auto & shape : shapes) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++) {
            auto fv = size_t(shape.mesh.num_face_vertices[f]);
            size_t i0 = shape.mesh.indices[index_offset + 0].vertex_index;
            size_t i1 = shape.mesh.indices[index_offset + 1].vertex_index;
            size_t i2 = shape.mesh.indices[index_offset + 2].vertex_index;
            mesh.faces.emplace_back(i0, i1, i2);
            index_offset += fv;
        }
    }
}

int main(int argc, char **argv)
{
    std::string input_path = std::string(argv[1]);
    std::string output_path = std::string(argv[2]);

    int dim = 256;
    float bw = 3.0, expand = 1.57735;
    std::vector<float> bbox{-0.5, -0.5, -0.5, 0.5, 0.5, 0.5};
    bool flag_silent = false, flag_fill = false, flag_full = false, flag_world = false,
         flag_dense = false, flag_unsigned = false, flag_index = false, flag_cube = true, flag_cell = false;

    openvdb::initialize();

    Mesh mesh;
    read_obj(input_path, mesh);

    // Set volume size and voxel size
    openvdb::BBoxd bbox_user(
            openvdb::Vec3d(bbox[0], bbox[1], bbox[2]),
            openvdb::Vec3d(bbox[3], bbox[4], bbox[5]));
    openvdb::Vec3d extent = bbox_user.extents();

    double length = openvdb::math::Max(
            extent.x(),
            extent.y(),
            extent.z());

    double voxel_size = length / (dim - 1);
    if (voxel_size < 1e-5) {
        std::cout << "The voxel size (" << voxel_size << ") is too small.";
        exit(1);
    }

    openvdb::Vec3d volume_size;
    volume_size.init(length, length, length);

    // Set narrow-band width
    float exbw = bw, inbw = bw;

    // // Transform to local grid space (cell-centered)
    openvdb::math::Transform::Ptr transform =
            openvdb::math::Transform::createLinearTransform(voxel_size);

    auto grid = openvdb::tools::meshToSignedDistanceField<openvdb::FloatGrid>(*transform, mesh.points, mesh.faces, std::vector<openvdb::Vec4I>(), exbw, inbw);

    const float outside = grid->background();
    const float width = 2.0f * outside;

    for (openvdb::FloatGrid::ValueOnIter iter = grid->beginValueOn(); iter; ++iter) {
        float dist = iter.getValue();
        iter.setValue((outside - dist) / width);
    }

    for (openvdb::FloatGrid::ValueOffIter iter = grid->beginValueOff(); iter; ++iter) {
        if (iter.getValue() < 0.0) {
            iter.setValue(1.0);
            iter.setValueOff();
        }
    }

    // Set output file
    openvdb::io::File(output_path).write({grid});
}