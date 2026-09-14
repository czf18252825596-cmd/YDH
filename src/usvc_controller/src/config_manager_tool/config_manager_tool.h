#ifndef CONFIG_MANAGER_TOOL_H
#define CONFIG_MANAGER_TOOL_H

#include <ros/ros.h>
#include <string>
#include <map>

class ConfigManagerTool {
private:
    static bool executeCommand(const std::string& command);
    static bool fileExists(const std::string& filename);
    static bool extractParamFromYaml(const std::string& config_file,
                                     const std::string& param_name,
                                     std::string& param_value);

public:
    // 字符串类型
    static bool updateParamInFile(const std::string& config_file,
                                  const std::string& param_name,
                                  const std::string& param_value);
    
    // 整数类型
    static bool updateParamInFile(const std::string& config_file,
                                  const std::string& param_name,
                                  int param_value);
    
    // 浮点数类型
    static bool updateParamInFile(const std::string& config_file,
                                  const std::string& param_name,
                                  double param_value);
    
    // 布尔类型
    static bool updateParamInFile(const std::string& config_file,
                                  const std::string& param_name,
                                  bool param_value);
    
    // 批量修改
    static bool updateParamsInFile(const std::string& config_file,
                                   const std::map<std::string, std::string>& params);
    
    // 验证参数值
    static bool verifyParamInFile(const std::string& config_file,
                                  const std::string& param_name,
                                  const std::string& param_value);
    
    // 读取参数（字符串）
    static bool getParamFromFile(const std::string& config_file,
                                 const std::string& param_name,
                                 std::string& param_value);
    
    // 读取参数（整数）
    static bool getParamFromFile(const std::string& config_file,
                                 const std::string& param_name,
                                 int& param_value);
    
    // 读取参数（浮点数）
    static bool getParamFromFile(const std::string& config_file,
                                 const std::string& param_name,
                                 double& param_value);
    
    // 读取参数（布尔值）
    static bool getParamFromFile(const std::string& config_file,
                                 const std::string& param_name,
                                 bool& param_value);
    
    // 检查参数是否存在
    static bool hasParameter(const std::string& config_file,
                             const std::string& param_name);
};

#endif