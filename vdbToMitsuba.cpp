//
// Created by andian on 2023/7/27.
//
#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/ValueTransformer.h>

#include <fstream>
#include <iostream>

void printUsage() {
    std::cout << "Usage: volconv filename.vdb [gridname (optional)]" << std::endl;
}

template<typename T>
void write(std::ofstream& f, T data) {
    f.write(reinterpret_cast<const char*>(&data), sizeof(data));
}

int main(int argc, char** argv) {
    if (argc == 1) {
        std::cerr << "Error: Too few arguments" << std::endl;
        printUsage();
        return -1;
    }

    std::string file_name = argv[1];
    std::string grid_name;
    if (argc == 3) grid_name = argv[2];

    std::string output_format = "binary";
    if (argc == 4) output_format = argv[3];


    openvdb::initialize();
    openvdb::io::File file(file_name);
    file.open();

    // Loop over all grids in the file and retrieve a shared pointer
    // to the one named "LevelSetSphere".  (This can also be done
    // more simply by calling file.readGrid("LevelSetSphere").)
    openvdb::GridBase::Ptr base_grid;
    for (openvdb::io::File::NameIterator name_iter = file.beginName();
         name_iter != file.endName(); ++name_iter) {
        // Read in only the grid we are interested in.
        if (grid_name.empty() || name_iter.gridName() == grid_name) {
            base_grid = file.readGrid(name_iter.gridName());
            if (grid_name.empty()) break;
        }
        else {
            std::cout << "skipping grid " << name_iter.gridName() << std::endl;
        }
    }
    openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>(base_grid);


    auto bbox_dim = grid->evalActiveVoxelDim();
    auto bbox = grid->evalActiveVoxelBoundingBox();

    auto ws_min = grid->indexToWorld(bbox.min());
    auto ws_max = grid->indexToWorld(bbox.max() - openvdb::Vec3R(1, 1, 1));

    std::swap(bbox_dim[2], bbox_dim[1]);
    std::swap(ws_min[2], ws_min[1]);
    std::swap(ws_max[2], ws_max[1]);

    auto ws_center = (ws_min + ws_max) * 0.5f;
    auto ws_scale = (ws_max - ws_min) * 0.5f;

//    printf("%.6f, %.6f, %.6f\n", ws_center[0], ws_center[1], ws_center[2]);
//    printf("%.6f, %.6f, %.6f\n", ws_scale[0], ws_scale[1], ws_scale[2]);

    openvdb::tools::GridSampler<openvdb::FloatGrid, openvdb::tools::BoxSampler> sampler(*grid);
    // Compute the value of the grid at fractional coordinates in index space.

    std::vector<openvdb::Vec3f> values;
    for (int k = bbox.min().y(); k < bbox.max().y(); ++k) {
        for (int j = bbox.min().z(); j < bbox.max().z(); ++j) {
            for (int i = bbox.min().x(); i < bbox.max().x(); ++i) {
                float value = sampler.isSample(openvdb::Vec3R(i, k, j));
                if (value == 1.0f) values.emplace_back(200.0, 200.0, 200.0);
                else values.emplace_back(0.1, 0.1, 0.1);
            }
        }
    }
    std::string output_filename = file_name.substr(0, file_name.find_last_of('.')) + ".vol";
    if (output_format == "ascii") {
        std::ofstream output_file(output_filename);
        if (!output_file.is_open()) {
            std::cout << "Could not open output file!\n";
            return -1;
        }
        for (size_t i = 0; i < values.size(); ++i) {
            if (i > 0) {
                output_file << ", ";
            }
            for (size_t j = 0; j < 3; ++j) {
                output_file << std::to_string(values[i][j]);
            }
        }
        output_file.close();
    }
    else if (output_format == "binary") {
        std::ofstream output_file(output_filename, std::ios::binary);
        if (!output_file.is_open()) {
            std::cout << "Could not open output file!\n";
            return -1;
        }
        output_file << 'V';
        output_file << 'O';
        output_file << 'L';
        uint8_t version = 3;
        write(output_file, version);

        write(output_file, (int32_t)1); // type
        write(output_file, (int32_t)bbox_dim.x() - 1);
        write(output_file, (int32_t)bbox_dim.y() - 1);
        write(output_file, (int32_t)bbox_dim.z() - 1);
        write(output_file, (int32_t)3); // #n channels

        auto xmin = float(ws_min.x());
        auto ymin = float(ws_min.y());
        auto zmin = float(ws_min.z());
        auto xmax = float(ws_max.x());
        auto ymax = float(ws_max.y());
        auto zmax = float(ws_max.z());
        write(output_file, xmin);
        write(output_file, ymin);
        write(output_file, zmin);
        write(output_file, xmax);
        write(output_file, ymax);
        write(output_file, zmax);

        for (auto & value : values) {
            for (size_t j = 0; j < 3; ++j) {
                write(output_file, value[j]);
            }
        }
        output_file.close();
    }
    else {
        std::cerr << "Invalid output format: " << output_format << std::endl;
        printUsage();
        return -1;
    }
    file.close();
}