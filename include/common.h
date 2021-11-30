//
// Created by Storm Phoenix on 2021/11/30.
//

#ifndef TEXTURINGPIPELINE_COMMON_H
#define TEXTURINGPIPELINE_COMMON_H

#include <spdlog/spdlog.h>

#define LOG_ERROR(...) spdlog::error(__VA_ARGS__)
#define LOG_WARN(...) spdlog::warn(__VA_ARGS__)
#define LOG_INFO(...) spdlog::info(__VA_ARGS__)
#define LOG_DEBUG(...) spdlog::debug(__VA_ARGS__)
#define LOG_TRACE(...) spdlog::trace(__VA_ARGS__)

#endif //TEXTURINGPIPELINE_COMMON_H
