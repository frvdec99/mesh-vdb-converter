//
// Created by andian on 2023/7/27.
//

#include "openvdb/openvdb.h"
#include "openvdb/tools/MeshToVolume.h"
#include "openvdb/tools/Dense.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <random>

const double eps = 1e-6;

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
        openvdb::Vec3s v = openvdb::Vec3s(vx, vy, vz);
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

std::default_random_engine engine;
std::uniform_real_distribution<float> uniform(0, 1);

float cal_area(const openvdb::BBoxd& bbox,
               const openvdb::Vec3s& v1, const openvdb::Vec3s& v2, const openvdb::Vec3s& v3,
               const openvdb::Vec3s& normal) {
    auto c1 = bbox.isInside(v1);
    auto c2 = bbox.isInside(v2);
    auto c3 = bbox.isInside(v3);
    auto area = normal.length() * 0.5f;

    if(c1 && c2 && c3) return area;
    int T = 100000;
    int s = 0;
    for(int i = 0; i < T; ++i) {
        auto u = uniform(engine);
        auto v = uniform(engine);
        v = v * (1 - u);
        auto p = v1 * u + v2 * v + v3 * (1 - u - v);
        if(bbox.isInside(p)) ++s;
    }
    return area * float(s) / float(T);
}

void cal_ndf(const Mesh& mesh, const openvdb::BBoxd& bbox, std::vector<std::pair<openvdb::Vec3s, float>>& ndf) {
    auto area_sum = 0.0f;
    for(const auto & face : mesh.faces) {
        openvdb::Vec3s v1 = mesh.points[face[0]];
        openvdb::Vec3s v2 = mesh.points[face[1]];
        openvdb::Vec3s v3 = mesh.points[face[2]];
        openvdb::Vec3s normal = (v2 - v1).cross(v3 - v1);
        auto area = cal_area(bbox, v1, v2, v3, normal);
        if(area > eps) {
            normal.normalize();
            ndf.emplace_back(normal, area);
            area_sum += area;
        }
    }
    for(auto & e : ndf) e.second /= area_sum;
}


int main(int argc, char **argv) {
    openvdb::initialize();
    std::string input_path = std::string(argv[1]);

    Mesh mesh;
    read_obj(input_path, mesh);

//    Point3f p_min(-0.3f, -0.9f, -0.3f);
//    Point3f p_max(0.3f, -0.5f, 0.3f);
    std::vector<float> bbox{-0.3, -0.9, -0.3, 0.3, -0.5, 0.3};
    openvdb::BBoxd bbox_ndf(
            openvdb::Vec3d(bbox[0], bbox[1], bbox[2]),
            openvdb::Vec3d(bbox[3], bbox[4], bbox[5]));

    std::vector<std::pair<openvdb::Vec3s, float>> ndf;
    cal_ndf(mesh, bbox_ndf, ndf);

    FILE* f;
    auto err = fopen_s(&f, argv[2], "w");
    fprintf(f, "%d\n", int(ndf.size()));
    for(const auto& e : ndf) {
        fprintf(f, "%.6f %.6f %.6f %.6f\n", e.first[0], e.first[1], e.first[2], e.second);
    }
    fclose(f);

    return 0;
}