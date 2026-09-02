// ONNX Runtime vendoring 확인 + export 한 정책의 입출력 규약 점검.
//
//   make ort-check
//
// 이 프로그램은 main.out 에 포함되지 않는다 (SOURCES 는 src/ 만 훑는다).
// 확인하는 것:
//   1) lib/onnxruntime 링크와 rpath 가 동작하는지
//   2) policy.onnx 의 입력 9개 · 출력 1개 이름과 형상이 명세와 맞는지
//   3) 세션 설정(스레드풀 1개)이 적용되는지, 추론이 도는지와 걸리는 시간
#include <onnxruntime_cxx_api.h>

#include "common/robot_config.hpp"

#include <array>
#include <chrono>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

namespace {

std::string shape_str(const std::vector<int64_t>& s) {
    std::string out = "[";
    for (size_t i = 0; i < s.size(); ++i) {
        out += std::to_string(s[i]);
        if (i + 1 < s.size()) out += ",";
    }
    return out + "]";
}

int64_t numel(const std::vector<int64_t>& s) {
    return std::accumulate(s.begin(), s.end(), int64_t{1}, std::multiplies<int64_t>());
}

}  // namespace

int main(int argc, char** argv) {
    const char* model = (argc > 1) ? argv[1] : "drumrobot_server/data/policy/policy.onnx";

    std::cout << "ORT " << Ort::GetVersionString() << "\n";
    std::cout << "model: " << model << "\n\n";

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ort_check");

    // 실시간 스레드에서 쓸 설정과 같게 (명세 06절)
    Ort::SessionOptions opt;
    opt.SetIntraOpNumThreads(1);
    opt.SetInterOpNumThreads(1);
    opt.SetExecutionMode(ORT_SEQUENTIAL);

    std::unique_ptr<Ort::Session> session;
    try {
        session = std::make_unique<Ort::Session>(env, model, opt);
    } catch (const Ort::Exception& e) {
        std::cerr << "세션 생성 실패: " << e.what() << "\n";
        return 1;
    }

    Ort::AllocatorWithDefaultOptions alloc;
    auto mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

    // ---- 입력 ----
    const size_t n_in = session->GetInputCount();
    std::vector<std::string> in_names;
    std::vector<std::vector<int64_t>> in_shapes;
    std::vector<std::vector<float>> in_bufs;

    std::cout << "입력 " << n_in << "개\n";
    for (size_t i = 0; i < n_in; ++i) {
        auto name = session->GetInputNameAllocated(i, alloc);
        auto shape = session->GetInputTypeInfo(i)
                         .GetTensorTypeAndShapeInfo().GetShape();
        std::cout << "  " << i << "  " << name.get() << "  " << shape_str(shape) << "\n";
        in_names.emplace_back(name.get());
        in_shapes.push_back(shape);
        in_bufs.emplace_back(static_cast<size_t>(numel(shape)), 0.0f);
    }

    // ---- 출력 ----
    const size_t n_out = session->GetOutputCount();
    std::vector<std::string> out_names;
    std::vector<std::vector<int64_t>> out_shapes;
    std::vector<std::vector<float>> out_bufs;

    std::cout << "\n출력 " << n_out << "개\n";
    for (size_t i = 0; i < n_out; ++i) {
        auto name = session->GetOutputNameAllocated(i, alloc);
        auto shape = session->GetOutputTypeInfo(i)
                         .GetTensorTypeAndShapeInfo().GetShape();
        std::cout << "  " << i << "  " << name.get() << "  " << shape_str(shape) << "\n";
        out_names.emplace_back(name.get());
        out_shapes.push_back(shape);
        out_bufs.emplace_back(static_cast<size_t>(numel(shape)), 0.0f);
    }

    // ---- 텐서를 우리 버퍼 위에 한 번만 만든다 (매 스텝 할당 회피) ----
    std::vector<Ort::Value> ins, outs;
    std::vector<const char*> in_cstr, out_cstr;
    for (size_t i = 0; i < n_in; ++i) {
        ins.push_back(Ort::Value::CreateTensor<float>(
            mem, in_bufs[i].data(), in_bufs[i].size(),
            in_shapes[i].data(), in_shapes[i].size()));
        in_cstr.push_back(in_names[i].c_str());
    }
    for (size_t i = 0; i < n_out; ++i) {
        outs.push_back(Ort::Value::CreateTensor<float>(
            mem, out_bufs[i].data(), out_bufs[i].size(),
            out_shapes[i].data(), out_shapes[i].size()));
        out_cstr.push_back(out_names[i].c_str());
    }

    // ---- 워밍업 후 측정 (명세 06절: 지연 초기화를 임계 경로 밖으로) ----
    Ort::RunOptions run_opt{nullptr};
    for (int i = 0; i < 50; ++i) {
        session->Run(run_opt, in_cstr.data(), ins.data(), n_in,
                     out_cstr.data(), outs.data(), n_out);
    }

    const int N = 2000;
    std::vector<double> us(N);
    for (int i = 0; i < N; ++i) {
        auto t0 = std::chrono::steady_clock::now();
        session->Run(run_opt, in_cstr.data(), ins.data(), n_in,
                     out_cstr.data(), outs.data(), n_out);
        us[i] = std::chrono::duration<double, std::micro>(
                    std::chrono::steady_clock::now() - t0).count();
    }
    std::sort(us.begin(), us.end());
    const double mean = std::accumulate(us.begin(), us.end(), 0.0) / N;

    std::cout << "\n추론 시간 (" << N << "회, 워밍업 50회 후)\n"
              << "  평균 " << mean << " us\n"
              << "  p50  " << us[N / 2] << " us\n"
              << "  p99  " << us[N * 99 / 100] << " us\n"
              << "  최대 " << us[N - 1] << " us\n"
              << "  예산 " << ROBOT::POLICY_DT_SECOND * 1e6
              << " us (정책 주기 " << ROBOT::POLICY_TICK_STRIDE << "틱) / 워치독 "
              << 3 * ROBOT::POLICY_DT_SECOND * 1e6 << " us\n";

    std::cout << "\n출력값 (입력 전부 0): ";
    for (float v : out_bufs[0]) std::cout << v << " ";
    std::cout << "\n";

    return 0;
}
