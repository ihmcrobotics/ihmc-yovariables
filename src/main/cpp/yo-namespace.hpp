#pragma once

#include <string>
#include <vector>
#include <numeric>
#include <sstream>
#include <memory>

namespace ihmc
{
class YoNamespace : public std::enable_shared_from_this<YoNamespace>
{
public:
    explicit YoNamespace(const std::string& name);
    explicit YoNamespace(const std::vector<std::string>& names);

    const std::string& get_name() const;

    const std::vector<std::string>& get_subnames() const;

    YoNamespace append(std::shared_ptr<YoNamespace> other);
    
    YoNamespace append(std::string& other);

    void check_sanity() const;

private:
    std::vector<std::string> subnames_;
    std::string name_;
};
} // namepsace ihmc
