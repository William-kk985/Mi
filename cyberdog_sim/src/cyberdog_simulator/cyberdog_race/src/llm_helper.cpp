#include "cyberdog_race/llm_helper.hpp"

#if defined(LLM_MODE_PROXY) || defined(LLM_MODE_API)

#ifdef LLM_MODE_API
#include <curl/curl.h>
#endif
#include <cstdio>
#include <cstring>

// ═══ PROXY: ROS2 Service ═══
#ifdef LLM_MODE_PROXY

bool LLMHelper::init(rclcpp::Node* node) {
    if (!node) return false;
    node_ = node;
    client_ = node_->create_client<cyberdog_race::srv::LLMAsk>(LLM_SERVICE_NAME);
    return client_->wait_for_service(std::chrono::seconds(5));
}

bool LLMHelper::ping() {
    return client_ && client_->service_is_ready();
}

std::string LLMHelper::ask(const std::string& prompt, int timeout_sec) {
    if (!client_ || !client_->service_is_ready()) return "";
    auto req = std::make_shared<cyberdog_race::srv::LLMAsk::Request>();
    req->prompt = prompt;
    auto future = client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(timeout_sec)) != std::future_status::ready) return "";
    auto resp = future.get();
    if (!resp->success) return "";
    return resp->response;
}

#endif  // LLM_MODE_PROXY

// ═══ API: libcurl（不变） ═══
#if defined(LLM_MODE_API)

bool LLMHelper::init(rclcpp::Node* node) {
    (void)node;
    url_ = LLM_DEFAULT_URL;
    model_ = LLM_DEFAULT_MODEL;
    load_api_config();
    return !url_.empty();
}

bool LLMHelper::ping() {
    return !ask("ping", 3).empty();
}

std::string LLMHelper::ask(const std::string& prompt, int timeout_sec) {
    std::string escaped;
    for (char c : prompt) {
        if (c == '"') escaped += "\\\"";
        else if (c == '\\') escaped += "\\\\";
        else if (c == '\n') escaped += "\\n";
        else escaped += c;
    }

    char body_buf[4096];
    int written = snprintf(body_buf, sizeof(body_buf),
        "{\"model\":\"%s\","
        "\"messages\":["
         "{\"role\":\"system\",\"content\":\"CyberDog control. Output JSON only: {\\\"action\\\":\\\"forward/back/left/right/stop/push/jump\\\",\\\"speed\\\":0-0.5,\\\"yaw\\\":-0.5-0.5}\"},"
         "{\"role\":\"user\",\"content\":\"%s\"}],"
        "\"temperature\":0.1,\"max_tokens\":200}",
        model_.c_str(), escaped.c_str());
    if (written < 0 || written >= (int)sizeof(body_buf)) {
        fprintf(stderr, "[LLM API] body_buf overflow, prompt too long\n");
        return "";
    }

    CURL* curl = curl_easy_init();
    if (!curl) return "";

    std::string response;
    curl_easy_setopt(curl, CURLOPT_URL, url_.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body_buf);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, (long)timeout_sec);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
        +[](void* ptr, size_t s, size_t n, void* user) -> size_t {
            ((std::string*)user)->append((char*)ptr, s*n);
            return s*n;
        });
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    if (!key_.empty()) {
        std::string auth = "Authorization: Bearer " + key_;
        headers = curl_slist_append(headers, auth.c_str());
    }
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    curl_easy_perform(curl);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    auto key_pos = response.find("\"content\":\"");
    if (key_pos == std::string::npos) return "";
    key_pos += 11;
    auto end = response.find("\"", key_pos);
    return (end == std::string::npos) ? response.substr(key_pos)
                                       : response.substr(key_pos, end - key_pos);
}

void LLMHelper::load_api_config(const std::string& path) {
    FILE* f = fopen(path.c_str(), "r");
    if (!f) return;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char val[200] = {};
        if (sscanf(line, "url=%199s", val) == 1) url_ = val;
        else if (sscanf(line, "model=%199s", val) == 1) model_ = val;
        else if (sscanf(line, "api_key=%199s", val) == 1) key_ = val;
    }
    fclose(f);
}

#endif  // LLM_MODE_API

#endif  // LLM_MODE_PROXY || LLM_MODE_API
