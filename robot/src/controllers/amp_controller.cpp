#include <stdexcept>

#include <onnxruntime_cxx_api.h>

#include "robot/controllers/amp_controller.hpp"

namespace {

const std::size_t ACTION_SIZE = 12;

std::size_t shape_size(std::vector<int64_t>& shape) {
    std::size_t n = 1;
    for (auto& d : shape) {
        if (d < 0) d = 1;
        n *= static_cast<std::size_t>(d);
    }
    return n;
}

}

struct controller::AmpController::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "AmpController"};
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    Ort::MemoryInfo memory_info{
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)
    };
    
    std::string input_name;
    std::string output_name;       // joint position targets
    std::string actions_name;      // raw actions, empty if the model has a single output
    std::vector<int64_t> input_shape;
    std::vector<int64_t> output_shape;
    std::vector<float> input;
    std::vector<float> output;
};

controller::AmpController::AmpController(const std::string& model_file)
: model_file_(model_file) {}

controller::AmpController::~AmpController() {
    shutdown();
}

void controller::AmpController::initialize() {
    if (initialized_) return;

    if (model_file_.empty()) {
        throw std::runtime_error("[AmpController::initialize] Empty model file.");
    }

    impl_ = std::make_unique<Impl>();
    impl_->session_options.SetIntraOpNumThreads(1);
    impl_->session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    impl_->session = std::make_unique<Ort::Session>(
        impl_->env,
        model_file_.c_str(),
        impl_->session_options
    );

    Ort::AllocatorWithDefaultOptions allocator;

    const auto input_name = impl_->session->GetInputNameAllocated(0, allocator);
    impl_->input_name = input_name.get();

    // arti_h1 scripts/pt_to_onnx.py exports ("actions", "q_des"): send q_des to the motors, feed actions back.
    std::size_t output_index = 0;
    const std::size_t n_outputs = impl_->session->GetOutputCount();
    for (std::size_t i = 0; i < n_outputs; ++i) {
        const auto name = impl_->session->GetOutputNameAllocated(i, allocator);
        const std::string value = name.get();
        if (value == "q_des") {
            output_index = i;
        } else if (value == "actions") {
            impl_->actions_name = value;
        }
    }
    const auto output_name = impl_->session->GetOutputNameAllocated(output_index, allocator);
    impl_->output_name = output_name.get();
    if (impl_->actions_name == impl_->output_name) impl_->actions_name.clear();

    impl_->input_shape = impl_->session->GetInputTypeInfo(0)
        .GetTensorTypeAndShapeInfo().GetShape();
    impl_->output_shape = impl_->session->GetOutputTypeInfo(output_index)
        .GetTensorTypeAndShapeInfo().GetShape();
    
    impl_->input.assign(shape_size(impl_->input_shape), 0.0f);
    impl_->output.assign(shape_size(impl_->output_shape), 0.0f);
    
    initialized_ = true;
}

void controller::AmpController::shutdown() {
    impl_.reset();
    initialized_ = false;
}

void controller::AmpController::update(const std::vector<double>& observation, std::vector<double>& action) {
    if (!initialized_ || !impl_ || !impl_->session) {
        throw std::runtime_error("[AmpController::update] Not initialized.");
    }

    if (observation.size() != impl_->input.size()) {
        throw std::runtime_error("[AmpController::update] Observation size mismatch.");
    }

    for (std::size_t i = 0; i < observation.size(); ++i) {
        impl_->input[i] = static_cast<float>(observation[i]);
    }

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        impl_->memory_info,
        impl_->input.data(),
        impl_->input.size(),
        impl_->input_shape.data(),
        impl_->input_shape.size()
    );

    const char* input_names[] = { impl_->input_name.c_str() };
    const char* output_names[] = { impl_->output_name.c_str(), impl_->actions_name.c_str() };
    const std::size_t n_outputs = impl_->actions_name.empty() ? 1 : 2;

    auto output_tensors = impl_->session->Run(
        Ort::RunOptions{nullptr},
        input_names,
        &input_tensor,
        1,
        output_names,
        n_outputs
    );

    const float* out = output_tensors[0].GetTensorData<float>();
    action.resize(impl_->output.size());
    for (std::size_t i = 0; i < action.size(); ++i) {
        action[i] = static_cast<double>(out[i]);
    }

    policy_actions_.clear();
    if (n_outputs == 2) {
        const float* raw = output_tensors[1].GetTensorData<float>();
        policy_actions_.resize(impl_->output.size());
        for (std::size_t i = 0; i < policy_actions_.size(); ++i) {
            policy_actions_[i] = static_cast<double>(raw[i]);
        }
    }
}