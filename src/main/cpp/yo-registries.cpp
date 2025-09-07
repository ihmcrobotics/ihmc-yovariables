#include "yo-registries.hpp"

std::string YoTools::join(const std::vector<std::string>& elements, const std::string& separator)    
{
    if (elements.empty())
        return "";

    return std::accumulate(std::next(elements.begin()), elements.end(), elements[0],
                           [&separator](const std::string& a, const std::string& b) {
                               return a + separator + b;
                           });
}

std::vector<std::string> YoTools::split(const std::string& str, const char& delimiter)
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

YoRegistry::YoRegistry(const std::string& name) : name_(name) {}

void YoRegistry::add_namespace(const std::shared_ptr<YoNamespace>& yoNamespace)
{
    namespaces_.push_back(yoNamespace);
}

const std::string& YoRegistry::get_name() const 
{
    return name_;
}

const std::vector<std::shared_ptr<YoNamespace>>& YoRegistry::get_namespaces() const
{
    return namespaces_;
}

void YoRegistry::remove_variable(const std::shared_ptr<YoVariable>& variable)
{
    if (!name_to_variable_map_.count(variable->get_name()))
        return;

    std::string var_name = variable->get_name();
    std::transform(var_name.begin(), var_name.end(), var_name.begin(), ::tolower);

    // TODO check restriction level
    variable->clear_registry();
    // remove from the lists
    // TODO check if this removal is correct
    variables_.erase(std::remove(variables_.begin(), variables_.end(), variable), variables_.end());
    name_to_variable_map_.erase(var_name);

    std::shared_ptr<YoRegistry> nullPtr;
    notify_listeners(this->shared_from_this(), nullPtr, variable, ChangeType::VARIABLE_REMOVED);
}

void YoRegistry::add_variable(const std::shared_ptr<YoVariable>& variable)
{
    // TODO check restriction level

    if (name_to_variable_map_.count(variable->get_name()))
    {
        return;
    }

    std::string var_name = variable->get_name();
    std::transform(var_name.begin(), var_name.end(), var_name.begin(), ::tolower);

    if (name_to_variable_map_.count(var_name))
    {
        throw std::runtime_error("Name collision for new variable: " + var_name + ". Parent name space = " + get_name());
    }

    if (variable->get_registry() != nullptr)
    {
        variable->get_registry()->remove_variable(variable);
    }

    variables_.push_back(variable);
    name_to_variable_map_[var_name] = variable;
    variable->set_registry(this->shared_from_this());

    std::shared_ptr<YoRegistry> nullPtr;
    notify_listeners(this->shared_from_this(), nullPtr, variable, ChangeType::VARIABLE_ADDED);
}

void YoRegistry::notify_listeners(const std::shared_ptr<YoRegistry>& target_parent_registry, const std::shared_ptr<YoRegistry>& registry, const std::shared_ptr<YoVariable>& variable, const ChangeType change_type)
{
    // Placeholder for notifying listeners about changes
    // Actual implementation would depend on how listeners are managed
}


// Implement functions declared in yo-registries.h here
