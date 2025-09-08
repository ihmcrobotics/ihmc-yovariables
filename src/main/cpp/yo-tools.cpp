#include "yo-tools.hpp"

namespace ihmc
{

namespace YoTools
{

const std::string NAMESPACE_SEPARATOR = ".";
const char NAMESPACE_DELIMITER = '.';


std::string join(const std::vector<std::string>& elements, const std::string& separator)
{
    if (elements.empty())
        return "";

    return std::accumulate(std::next(elements.begin()), elements.end(), elements[0],
                           [&separator](const std::string& a, const std::string& b) {
                               return a + separator + b;
                           });
}

std::vector<std::string> split(const std::string& str, const char& delimiter)
{
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, delimiter))
    {
        if (!item.empty())
            tokens.push_back(item);
    }
    return tokens;
} 

YoNamespace concatenate_namespaces(std::shared_ptr<const YoNamespace> namespaceA, std::shared_ptr<YoNamespace> namespaceB)
{
    std::vector<std::string> sub_names;
    sub_names.insert(sub_names.end(), namespaceA->get_subnames().begin(), namespaceA->get_subnames().end());
    sub_names.insert(sub_names.end(), namespaceB->get_subnames().begin(), namespaceB->get_subnames().end());

    return YoNamespace(sub_names);
}

YoNamespace concatenate_space_and_name(std::shared_ptr<const YoNamespace> nameSpace, const std::string& name)
{
    std::vector<std::string> split_names = split(name, NAMESPACE_DELIMITER);
    std::vector<std::string> sub_names;
    sub_names.insert(sub_names.end(), nameSpace->get_subnames().begin(), nameSpace->get_subnames().end());
    sub_names.insert(sub_names.end(), split_names.begin(), split_names.end());

    return YoNamespace(sub_names);
}

YoNamespace concatenate_name_and_space(const std::string& name, std::shared_ptr<const YoNamespace> nameSpace)
{
    std::vector<std::string> split_names = split(name, NAMESPACE_DELIMITER);
    std::vector<std::string> sub_names;
    sub_names.insert(sub_names.end(), split_names.begin(), split_names.end());
    sub_names.insert(sub_names.end(), nameSpace->get_subnames().begin(), nameSpace->get_subnames().end());

    return YoNamespace(sub_names);
}

YoNamespace concatenate_names(const std::string& nameA, const std::string& nameB)
{
    std::vector<std::string> split_namesA = split(nameA, NAMESPACE_DELIMITER);
    std::vector<std::string> split_namesB = split(nameB, NAMESPACE_DELIMITER);

    std::vector<std::string> sub_names;
    sub_names.insert(sub_names.end(), split_namesA.begin(), split_namesA.end());
    sub_names.insert(sub_names.end(), split_namesB.begin(), split_namesB.end());

    return YoNamespace(sub_names);
}

void check_namespace_sanity(std::shared_ptr<YoNamespace> nameSpace)
{
    std::vector<std::string> sub_names = nameSpace->get_subnames();
    if (std::any_of(sub_names.begin(), sub_names.end(), [](const std::string& s) { 
                                      return s.empty(); 
                                  }))
    {
        throw std::runtime_error("The namespace has 1+ empty subname. Namespace: " + nameSpace->get_name());
    }

    if (std::find(sub_names.begin(), sub_names.end(), NAMESPACE_SEPARATOR) == sub_names.end())
    {
        throw std::runtime_error("A sub-name cannot contain the separator string '.'.");
    }

    std::string joined_subnames = join(nameSpace->get_subnames(), NAMESPACE_SEPARATOR);

    if (joined_subnames != nameSpace->get_name())
    {
        throw std::runtime_error("The namespace has inconsistent sub-names. Namespace " + nameSpace->get_name() + " Sub-names: " + joined_subnames);
    }
}

}
} // namespace ihmc