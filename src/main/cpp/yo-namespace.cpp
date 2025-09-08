#include "yo-namespace.hpp"
#include "yo-tools.hpp"

namespace ihmc
{
YoNamespace::YoNamespace(const std::string& name)
{
    if (name.empty())
        throw std::invalid_argument("YoNamespace name cannot be empty.");
    name_ = name;
    subnames_ = YoTools::split(name, '.');
    for (const auto& subname : subnames_)
    {
        if (subname.empty())
        {
            std::ostringstream oss;
            oss << "Cannot create a namespace with empty sub-names: ";
            for (size_t i = 0; i < subnames_.size(); ++i)
            {
                oss << "\"" << subnames_[i] << "\"";
                if (i != subnames_.size() - 1)
                    oss << ", ";
            }
            throw std::invalid_argument(oss.str());
        }
    }
}

YoNamespace::YoNamespace(const std::vector<std::string>& names)
{
    if (names.empty())
        throw std::invalid_argument("YoNamespace name cannot be empty.");
    name_ = YoTools::join(names, ".");
    subnames_ = names;
}

const std::string& YoNamespace::get_name() const 
{ 
    return name_;
}

const std::vector<std::string>& YoNamespace::get_subnames() const 
{ 
    return subnames_;
}

YoNamespace YoNamespace::append(std::shared_ptr<YoNamespace> other)
{
    return YoTools::concatenate_namespaces(this->shared_from_this(), other);
} 

YoNamespace YoNamespace::append(std::string& other)
{
    return YoTools::concatenate_space_and_name(this->shared_from_this(), other);
} 
}