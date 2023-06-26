//
// Created by ola on 6/24/23.
//
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <utility>
#include <boost/filesystem.hpp>

#include <colmap/util/option_manager.h>
#include <colmap/util/string.h>
#include <colmap/base/image_reader.h>
#include <colmap/base/database_cache.h>
#include <colmap/base/reconstruction.h>
#include <colmap/feature/matching.h>
#include <colmap/feature/extraction.h>
#include <colmap/controllers/automatic_reconstruction.h>
#include <colmap/controllers/incremental_mapper.h>

#include <colmap/util/misc.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

void run_image_registrator(std::string &input_path, std::string &output_path, std::string &db_path);

void run_vocab_tree_matcher(std::string &vocab_tree_path, std::string &output_path, std::string &db_path);

void run_feature_extractor(std::string &img_path, std::string &img_list, std::string &db_path);

void run_model_converter(std::string &input_path, std::string &output_path, std::string &output_type);


int main(int argc, char **argv) {
//    std::string pkgpath = ament_index_cpp::get_package_share_directory("colmap_localization");

    // used this because the ament refers to the clmap_localization in the install directory
    std::string pkgpath = "/home/ola/smart_home/src/smart-home/external/ros2_colmap/colmap_localization";

//    std::string sourcePath = pkgpath + "/image_registering/0118.jpg " ;
//    std::string destinationFolder = pkgpath + "/image_registering/images" ;


    std::string img_path = pkgpath + "/image_registering/images2";
    std::string input_path = pkgpath + "/image_registering/sparse/0";
    std::string vocab_tree_path = pkgpath + "/image_registering/vocab-tree.bin";
    std::string output_path = pkgpath + "/image_registering/registered";
    std::string db_path = pkgpath + "/image_registering/database.db";
    std::string img_list = pkgpath + "/image_registering/image-list.txt";
    std::string output_path_converter = pkgpath + "/image_registering/sparse_registered_txt";
    std::string output_type = "TXT";

    if (true) {
        colmap::ReconstructionManager reconstruction_manager;

        colmap::AutomaticReconstructionController::Options options;
        options.workspace_path = pkgpath + "/image_registering";
        options.image_path = "/home/ola/Desktop/img_lab";
        options.data_type = colmap::AutomaticReconstructionController::DataType::VIDEO;
        options.vocab_tree_path = vocab_tree_path;
        // Set other options as needed
        colmap::AutomaticReconstructionController automatic_reconstruction_controller(options, &reconstruction_manager);

//     Run the reconstruction
        automatic_reconstruction_controller.Start();

        // Wait for the reconstruction to finish
        automatic_reconstruction_controller.Wait();

        // output is database.db and sparse folder
    } else  {
        run_feature_extractor(img_path, img_list, db_path);
        // needs image-list.txt that includes the name fo the new images added and the mage itself outputs nothing just changes the database

        run_vocab_tree_matcher(vocab_tree_path, img_list, db_path);
        // needs vocab-tree.bin that you have to have outputs nothing just changes the database

        // Create the destination folder
        boost::filesystem::create_directory(pkgpath + "/image_registering/registered");

        run_image_registrator(input_path, output_path, db_path);
        // need an empty registered folder
//    if (false) {
        boost::filesystem::create_directory(pkgpath + "/image_registering/sparse_registered_txt");
        run_model_converter(output_path, output_path_converter,
                            output_type); // input of this is the output of ime_registrator
    }


        return 0;


}

void run_image_registrator(std::string &input_path, std::string &output_path, std::string &db_path) {
    colmap::OptionManager options;
    options.AddDatabaseOptions();
    options.AddRequiredOption("input_path", &input_path);
    options.AddRequiredOption("output_path", &output_path);
    options.AddMapperOptions();
    *options.database_path = db_path;

    colmap::DatabaseCache database_cache;

    {
        colmap::Database database(*options.database_path);
        colmap::Timer timer;
        timer.Start();
        const size_t min_num_matches =
                static_cast<size_t>(options.mapper->min_num_matches);
        database_cache.Load(database,
                            min_num_matches,
                            options.mapper->ignore_watermarks,
                            options.mapper->image_names);
        std::cout << std::endl;
        timer.PrintMinutes();
    }

    std::cout << std::endl;

    colmap::Reconstruction reconstruction;
    reconstruction.Read(input_path);

    colmap::IncrementalMapper mapper(&database_cache);
    mapper.BeginReconstruction(&reconstruction);

    const auto mapper_options = options.mapper->Mapper();

    for (const auto &image: reconstruction.Images()) {
        if (image.second.IsRegistered()) {
            continue;
        }

        std::cout << ("Registering image #" + std::to_string(image.first) + " (" +
                      std::to_string(reconstruction.NumRegImages() + 1) + ")");

        std::cout << "  => Image sees " << image.second.NumVisiblePoints3D()
                  << " / " << image.second.NumObservations() << " points"
                  << std::endl;

        mapper.RegisterNextImage(mapper_options, image.first);
    }

    const bool kDiscardReconstruction = false;
    mapper.EndReconstruction(kDiscardReconstruction);
    reconstruction.Write(output_path);

    auto cams = reconstruction.Images();
}

bool VerifySiftGPUParams(const bool use_gpu) {
    return true;
#if !defined(CUDA_ENABLED) && !defined(OPENGL_ENABLED)
    if (use_gpu) {
        std::cerr << "ERROR: Cannot use Sift GPU without CUDA or OpenGL support; "
                     "set SiftExtraction.use_gpu or SiftMatching.use_gpu to false."
                  << std::endl;
        return false;
    }
#endif
    return true;
}

void run_vocab_tree_matcher(std::string &vocab_tree_path, std::string &img_list, std::string &db_path) {
    colmap::OptionManager options;
    options.AddDatabaseOptions();
    options.AddVocabTreeMatchingOptions();
    *options.database_path = db_path;
    options.vocab_tree_matching->vocab_tree_path = vocab_tree_path;
    options.vocab_tree_matching->match_list_path = img_list;
    options.sift_matching->use_gpu = true;

    if (!VerifySiftGPUParams(options.sift_matching->use_gpu)) {
        return;
    }

    colmap::VocabTreeFeatureMatcher feature_matcher(*options.vocab_tree_matching,
                                                    *options.sift_matching,
                                                    *options.database_path);


    feature_matcher.Start();
    feature_matcher.Wait();

}

enum class CameraMode {
    AUTO = 0, SINGLE = 1, PER_FOLDER = 2, PER_IMAGE = 3
};

bool VerifyCameraParams(const std::string &camera_model,
                        const std::string &params) {
    if (!colmap::ExistsCameraModelWithName(camera_model)) {
        std::cerr << "ERROR: Camera model does not exist" << std::endl;
        return false;
    }

    const std::vector<double> camera_params = colmap::CSVToVector<double>(params);
    const int camera_model_id = colmap::CameraModelNameToId(camera_model);

    if (camera_params.size() > 0 &&
        !colmap::CameraModelVerifyParams(camera_model_id, camera_params)) {
        std::cerr << "ERROR: Invalid camera parameters" << std::endl;
        return false;
    }
    return true;
}


void UpdateImageReaderOptionsFromCameraMode(colmap::ImageReaderOptions &options,
                                            CameraMode mode) {
    switch (mode) {
        case CameraMode::AUTO:
            options.single_camera = false;
            options.single_camera_per_folder = false;
            options.single_camera_per_image = false;
            break;
        case CameraMode::SINGLE:
            options.single_camera = true;
            options.single_camera_per_folder = false;
            options.single_camera_per_image = false;
            break;
        case CameraMode::PER_FOLDER:
            options.single_camera = false;
            options.single_camera_per_folder = true;
            options.single_camera_per_image = false;
            break;
        case CameraMode::PER_IMAGE:
            options.single_camera = false;
            options.single_camera_per_folder = false;
            options.single_camera_per_image = true;
            break;
    }
}

void run_feature_extractor(std::string &img_path, std::string &img_list, std::string &db_path) {
    int camera_mode = -1;
    std::string descriptor_normalization = "l1_root";

    colmap::OptionManager options;
    options.AddDatabaseOptions();
    options.AddImageOptions();
    options.AddDefaultOption("camera_mode", &camera_mode);
    options.AddDefaultOption("image_list_path", &img_list);
    options.AddDefaultOption("descriptor_normalization",
                             &descriptor_normalization,
                             "{'l1_root', 'l2'}");
    *options.image_path = img_path;
    *options.database_path = db_path;

    colmap::ImageReaderOptions reader_options = *options.image_reader;
    reader_options.database_path = *options.database_path;
    reader_options.image_path = *options.image_path;

    if (camera_mode >= 0) {
        UpdateImageReaderOptionsFromCameraMode(reader_options,
                                               (CameraMode) camera_mode);
    }

    colmap::StringToLower(&descriptor_normalization);
    if (descriptor_normalization == "l1_root") {
        options.sift_extraction->normalization =
                colmap::SiftExtractionOptions::Normalization::L1_ROOT;
    } else if (descriptor_normalization == "l2") {
        options.sift_extraction->normalization =
                colmap::SiftExtractionOptions::Normalization::L2;
    } else {
        std::cerr << "ERROR: Invalid `descriptor_normalization`" << std::endl;
        return;
    }

    if (!img_list.empty()) {
        reader_options.image_list = colmap::ReadTextFileLines(img_list);
        if (reader_options.image_list.empty()) {
            return;
        }
    }

    if (!colmap::ExistsCameraModelWithName(reader_options.camera_model)) {
        std::cerr << "ERROR: Camera model does not exist" << std::endl;
    }

    if (!VerifyCameraParams(reader_options.camera_model,
                            reader_options.camera_params)) {
        return;
    }

    if (!VerifySiftGPUParams(options.sift_extraction->use_gpu)) {
        return;
    }


    colmap::SiftFeatureExtractor feature_extractor(reader_options,
                                                   *options.sift_extraction);

    feature_extractor.Start();
    feature_extractor.Wait();
}

void run_model_converter(std::string &input_path, std::string &output_path, std::string &output_type) {

    bool skip_distortion = false;

    colmap::OptionManager options;
    options.AddRequiredOption("input_path", &input_path);
    options.AddRequiredOption("output_path", &output_path);
    options.AddRequiredOption("output_type",
                              &output_path,
                              "{BIN, TXT, NVM, Bundler, VRML, PLY, R3D, CAM}");
    options.AddDefaultOption("skip_distortion", &skip_distortion);

    colmap::Reconstruction reconstruction;
    reconstruction.Read(input_path);

    colmap::StringToLower(&output_type);
    if (output_type == "bin") {
        reconstruction.WriteBinary(output_path);
    } else if (output_type == "txt") {
        reconstruction.WriteText(output_path);
    } else if (output_type == "nvm") {
        reconstruction.ExportNVM(output_path, skip_distortion);
    } else if (output_type == "bundler") {
        reconstruction.ExportBundler(output_path + ".bundle.out",
                                     output_path + ".list.txt",
                                     skip_distortion);
    } else if (output_type == "r3d") {
        reconstruction.ExportRecon3D(output_path, skip_distortion);
    } else if (output_type == "cam") {
        reconstruction.ExportCam(output_path, skip_distortion);
    } else if (output_type == "ply") {
        reconstruction.ExportPLY(output_path);
    } else if (output_type == "vrml") {
        const auto base_path = output_path.substr(0, output_path.find_last_of('.'));
        reconstruction.ExportVRML(base_path + ".images.wrl",
                                  base_path + ".points3D.wrl",
                                  1,
                                  Eigen::Vector3d(1, 0, 0));
    } else {
        std::cerr << "ERROR: Invalid `output_type`" << std::endl;
        return;
    }

}