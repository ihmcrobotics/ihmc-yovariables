#pragma once

#include <string>
#include <vector>
#include <numeric>
#include <sstream>
#include <memory>
#include "yo-namespace.hpp"

namespace ihmc
{

namespace YoTools
{

    void check_namespace_sanity(std::shared_ptr<YoNamespace> nameSpace);

    std::string join(const std::vector<std::string>& elements, const std::string& separator);

    std::vector<std::string> split(const std::string& str, const char& delimiter);

    YoNamespace concatenate_namespaces(std::shared_ptr<const YoNamespace> namespaceA, std::shared_ptr<YoNamespace> namespaceB);
    
    YoNamespace concatenate_space_and_name(std::shared_ptr<const YoNamespace> nameSpace, std::string& name);

    YoNamespace concatenate_name_and_space(std::string& name, std::shared_ptr<const YoNamespace> nameSpace);

    YoNamespace concatenate_names(std::string& nameA, std::string& nameB);

}

} // namespace ihmc