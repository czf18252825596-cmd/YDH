// usvc_controller/src/config_manager_tool.cpp
#include "config_manager_tool.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <sys/file.h>
#include <unistd.h>
#include <vector>
#include <stack>

// 去除字符串首尾空白
static std::string trim(const std::string& str) {
    if (str.empty()) return str;
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
}

// 检查字符串是否为数字（整数或浮点数）
static bool isNumeric(const std::string& str) {
    if (str.empty()) return false;
    char* endptr;
    strtod(str.c_str(), &endptr);
    return (*endptr == '\0');
}

// 检查字符串是否为布尔值
static bool isBoolean(const std::string& str) {
    std::string lower = str;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    return (lower == "true" || lower == "false" || 
            lower == "yes" || lower == "no" || 
            lower == "on" || lower == "off");
}

// 检查字符串是否需要引号（包含特殊字符或空格）
static bool needsQuotes(const std::string& str) {
    if (str.empty()) return true;
    
    // 检查是否包含YAML特殊字符或空格
    for (char c : str) {
        if (c == ':' || c == '#' || c == '@' || c == '`' || 
            c == '*' || c == '&' || c == '{' || c == '}' || 
            c == '[' || c == ']' || c == ',' || c == '>' || 
            c == '|' || c == '"' || c == '\'' || c == '\\' ||
            c == ' ' || c == '\t') {
            return true;
        }
    }
    return false;
}

// 为字符串添加引号（如果需要）
static std::string quoteIfNeeded(const std::string& str, bool isStringType) {
    // 如果是字符串类型（通过函数参数传入），或者不是数字/布尔值，并且需要引号
    if (isStringType || (!isNumeric(str) && !isBoolean(str) && needsQuotes(str))) {
        // 如果已经带有引号，就不再添加
        if ((str.front() == '"' && str.back() == '"') ||
            (str.front() == '\'' && str.back() == '\'')) {
            return str;
        }
        // 添加双引号
        return "\"" + str + "\"";
    }
    return str;
}

// 检查文件是否存在
bool ConfigManagerTool::fileExists(const std::string& filename) {
    std::ifstream f(filename.c_str());
    return f.good();
}

// 解析YAML路径并找到对应的行号
static bool findYamlPathInLines(const std::vector<std::string>& lines,
                               const std::string& param_name,
                               size_t& found_line_index,
                               std::string& indent_str) {
    
    // 分割路径
    std::vector<std::string> path_parts;
    std::stringstream ss(param_name);
    std::string part;
    while (std::getline(ss, part, '/')) {
        if (!part.empty()) {
            path_parts.push_back(part);
        }
    }
    
    if (path_parts.empty()) return false;
    
    // 使用栈来跟踪当前路径
    std::stack<std::string> current_path;
    std::stack<int> indent_levels;
    indent_levels.push(-1); // 根级别
    
    for (size_t i = 0; i < lines.size(); ++i) {
        const std::string& line = lines[i];
        std::string trimmed = trim(line);
        
        if (trimmed.empty() || trimmed[0] == '#') continue;
        
        // 计算缩进级别（空格数）
        size_t indent = line.find_first_not_of(" \t");
        if (indent == std::string::npos) indent = 0;
        
        // 检查是否包含冒号
        size_t colon_pos = trimmed.find(':');
        if (colon_pos == std::string::npos) continue;
        
        std::string key = trim(trimmed.substr(0, colon_pos));
        
        // 根据缩进调整当前路径
        while (!indent_levels.empty() && indent_levels.top() >= static_cast<int>(indent)) {
            if (!current_path.empty()) current_path.pop();
            indent_levels.pop();
        }
        
        // 检查是否匹配路径
        if (key == path_parts[current_path.size()]) {
            current_path.push(key);
            indent_levels.push(indent);
            
            // 如果完全匹配路径
            if (current_path.size() == path_parts.size()) {
                found_line_index = i;
                // 保存缩进
                indent_str = line.substr(0, indent);
                return true;
            }
        }
    }
    
    return false;
}

// 核心函数：更新参数到文件并同步到参数服务器
static bool updateParamInFileInternal(const std::string& config_file,
                                     const std::string& param_name,
                                     const std::string& param_value,
                                     bool isStringType = false) {
    
    ROS_DEBUG("[ConfigManager] Saving %s = %s to %s", 
              param_name.c_str(), param_value.c_str(), config_file.c_str());
    
    // ========== 1. 写入文件 ==========
    std::ifstream in_file(config_file);
    if (!in_file.is_open()) {
        ROS_ERROR("[ConfigManager] Cannot open file for reading: %s", config_file.c_str());
        return false;
    }
    
    int fd = open(config_file.c_str(), O_RDONLY);
    if (fd != -1) flock(fd, LOCK_SH);
    
    std::vector<std::string> lines;
    std::string line;
    bool found = false;
    
    while (std::getline(in_file, line)) {
        lines.push_back(line);
    }
    
    in_file.close();
    if (fd != -1) {
        flock(fd, LOCK_UN);
        close(fd);
    }
    
    // 查找YAML路径
    size_t line_index;
    std::string indent_str;
    if (!findYamlPathInLines(lines, param_name, line_index, indent_str)) {
        ROS_ERROR("[ConfigManager] Parameter '%s' not found in %s", 
                  param_name.c_str(), config_file.c_str());
        return false;
    }
    
    // 处理需要引号的值
    std::string processed_value = quoteIfNeeded(param_value, isStringType);
    
    // 获取参数名的最后一部分
    size_t last_slash = param_name.find_last_of('/');
    std::string last_param = (last_slash == std::string::npos) ? 
                              param_name : param_name.substr(last_slash + 1);
    
    // 更新该行
    std::string new_line = indent_str + last_param + ": " + processed_value;
    lines[line_index] = new_line;
    found = true;
    
    ROS_DEBUG("[ConfigManager] Updated line %zu: %s", line_index, new_line.c_str());
    
    // 写入文件
    std::ofstream out_file(config_file);
    if (!out_file.is_open()) {
        ROS_ERROR("[ConfigManager] Cannot open file for writing: %s", config_file.c_str());
        return false;
    }
    
    fd = open(config_file.c_str(), O_WRONLY);
    if (fd != -1) flock(fd, LOCK_EX);
    
    for (const auto& l : lines) {
        out_file << l << std::endl;
    }
    
    out_file.close();
    if (fd != -1) {
        flock(fd, LOCK_UN);
        close(fd);
    }
    
    if (!found) {
        ROS_ERROR("[ConfigManager] Failed to update parameter '%s'", param_name.c_str());
        return false;
    }
    
    // ========== 2. 同步到参数服务器 ==========
    ros::param::set(param_name, param_value); // 参数服务器保持原始值（不带引号）
    ROS_DEBUG("[ConfigManager] Synced %s to param server", param_name.c_str());
    
    return true;
}

// ========== 对外接口 ==========

bool ConfigManagerTool::updateParamInFile(const std::string& config_file,
                                         const std::string& param_name,
                                         const std::string& param_value) {
    // 字符串类型需要特殊处理引号
    return updateParamInFileInternal(config_file, param_name, param_value, true);
}

bool ConfigManagerTool::updateParamInFile(const std::string& config_file,
                                         const std::string& param_name,
                                         int param_value) {
    return updateParamInFileInternal(config_file, param_name, std::to_string(param_value), false);
}

bool ConfigManagerTool::updateParamInFile(const std::string& config_file,
                                         const std::string& param_name,
                                         double param_value) {
    return updateParamInFileInternal(config_file, param_name, std::to_string(param_value), false);
}

bool ConfigManagerTool::updateParamInFile(const std::string& config_file,
                                         const std::string& param_name,
                                         bool param_value) {
    return updateParamInFileInternal(config_file, param_name, 
                                     param_value ? "true" : "false", false);
}

bool ConfigManagerTool::updateParamsInFile(const std::string& config_file,
                                          const std::map<std::string, std::string>& params) {
    bool all_success = true;
    for (const auto& p : params) {
        // 对于map中的值，我们无法确定类型，默认按字符串处理（添加引号）
        if (!updateParamInFileInternal(config_file, p.first, p.second, true)) {
            all_success = false;
        }
    }
    return all_success;
}

// ========== 以下函数保持不变，但改进getParamFromFile以支持YAML路径 ==========

bool ConfigManagerTool::verifyParamInFile(const std::string& config_file,
                                         const std::string& param_name,
                                         const std::string& param_value) {
    std::string current_value;
    if (getParamFromFile(config_file, param_name, current_value)) {
        return current_value == param_value;
    }
    return false;
}

bool ConfigManagerTool::getParamFromFile(const std::string& config_file,
                                        const std::string& param_name,
                                        std::string& param_value) {
    
    std::ifstream file(config_file);
    if (!file.is_open()) {
        ROS_ERROR("[ConfigManager] Cannot open file for reading: %s", config_file.c_str());
        return false;
    }
    
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(file, line)) {
        lines.push_back(line);
    }
    file.close();
    
    // 查找YAML路径
    size_t line_index;
    std::string indent_str;
    if (!findYamlPathInLines(lines, param_name, line_index, indent_str)) {
        ROS_WARN("[ConfigManager] Parameter '%s' not found in %s", 
                 param_name.c_str(), config_file.c_str());
        return false;
    }
    
    // 提取参数值
    const std::string& target_line = lines[line_index];
    size_t colon_pos = target_line.find(':');
    if (colon_pos != std::string::npos) {
        std::string value_str = trim(target_line.substr(colon_pos + 1));
        
        // 去除引号
        if (value_str.size() >= 2) {
            if ((value_str.front() == '"' && value_str.back() == '"') ||
                (value_str.front() == '\'' && value_str.back() == '\'')) {
                value_str = value_str.substr(1, value_str.size() - 2);
            }
        }
        
        param_value = value_str;
        return true;
    }
    
    return false;
}

bool ConfigManagerTool::getParamFromFile(const std::string& config_file,
                                        const std::string& param_name,
                                        int& param_value) {
    std::string str_val;
    if (!getParamFromFile(config_file, param_name, str_val)) return false;
    try {
        param_value = std::stoi(str_val);
        return true;
    } catch (...) {
        return false;
    }
}

bool ConfigManagerTool::getParamFromFile(const std::string& config_file,
                                        const std::string& param_name,
                                        double& param_value) {
    std::string str_val;
    if (!getParamFromFile(config_file, param_name, str_val)) return false;
    try {
        param_value = std::stod(str_val);
        return true;
    } catch (...) {
        return false;
    }
}

bool ConfigManagerTool::getParamFromFile(const std::string& config_file,
                                        const std::string& param_name,
                                        bool& param_value) {
    std::string str_val;
    if (!getParamFromFile(config_file, param_name, str_val)) return false;
    
    std::string lower = str_val;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    
    param_value = (lower == "true" || lower == "1" || lower == "yes" || lower == "on");
    return true;
}

bool ConfigManagerTool::hasParameter(const std::string& config_file,
                                    const std::string& param_name) {
    std::string dummy;
    return getParamFromFile(config_file, param_name, dummy);
}

bool ConfigManagerTool::executeCommand(const std::string& command) {
    ROS_WARN("[ConfigManager] executeCommand is deprecated, using direct file access instead");
    int result = std::system(command.c_str());
    return result == 0;
}

bool ConfigManagerTool::extractParamFromYaml(const std::string& config_file,
                                            const std::string& param_name,
                                            std::string& param_value) {
    return getParamFromFile(config_file, param_name, param_value);
}