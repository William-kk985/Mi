#pragma once

#include "cyberdog_race/debug_config.hpp"

#if defined(LLM_MODE_PROXY) || defined(LLM_MODE_API)

#include <memory>
#include <string>

// rclcpp 本包始终依赖，这里无条件包含
#include <rclcpp/rclcpp.hpp>

#ifdef LLM_MODE_PROXY
#include <cyberdog_race/srv/llm_ask.hpp>
#endif

class LLMHelper {
public:
    LLMHelper() = default;
    ~LLMHelper() = default;

    bool init(rclcpp::Node* node = nullptr);
    std::string ask(const std::string& prompt, int timeout_sec = LLM_TIMEOUT);
    bool ping();

private:
#ifdef LLM_MODE_PROXY
    rclcpp::Node* node_{nullptr};
    rclcpp::Client<cyberdog_race::srv::LLMAsk>::SharedPtr client_;
#endif
#ifdef LLM_MODE_API
    std::string url_, key_, model_;
    void load_api_config(const std::string& path = "llm_config.conf");
#endif
};

#endif  // LLM_MODE_PROXY || LLM_MODE_API
