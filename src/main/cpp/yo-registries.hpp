#pragma once

#include <string>
#include <vector>
#include <numeric>
#include <sstream>
#include <memory>
#include <unordered_map>

namespace YoTools
{
    std::string join(const std::vector<std::string>& elements, const std::string& separator);

    std::vector<std::string> split(const std::string& str, const char& delimiter);
}// namespace YoTools

class YoNamespace
{
public:
    explicit YoNamespace(const std::string& name);
    explicit YoNamespace(const std::vector<std::string>& names);

private:

    const std::string& get_name() const;

    std::vector<std::string> subnames_;
    std::string name_;
};



// Forward declare this so it can be used in YoRegistry
class YoVariable;

class YoRegistry : public std::enable_shared_from_this<YoRegistry>
{
public:
    explicit YoRegistry(const std::string& name);

    void add_namespace(const std::shared_ptr<YoNamespace>& yoNamespace);

    const std::string& get_name() const;

    const std::vector<std::shared_ptr<YoNamespace>>& get_namespaces() const;

    void add_variable(const std::shared_ptr<YoVariable>& variable);

    void remove_variable(const std::shared_ptr<YoVariable>& variable);

    std::shared_ptr<YoVariable>& get_child(const std::string& name) const;

private:

    enum class ChangeType
    {
        REGISTRY_ADDED,
        REGISTRY_REMOVED,
        VARIABLE_ADDED,
        VARIABLE_REMOVED,
        CLEARED
    };

    std::string name_;
    std::vector<std::shared_ptr<YoNamespace>> namespaces_;
    std::vector<std::shared_ptr<YoVariable>> variables_;
    std::unordered_map<std::string, std::shared_ptr<YoVariable>> name_to_variable_map_;

    void notify_listeners(const std::shared_ptr<YoRegistry>& target_parent_registry, 
                          const std::shared_ptr<YoRegistry>& registry,
                          const std::shared_ptr<YoVariable>& variable, 
                          const ChangeType change_type);
};

class YoVariable : public std::enable_shared_from_this<YoVariable>
{
public:
    YoVariable(const std::string& name, std::shared_ptr<YoRegistry>& registry);

    virtual ~YoVariable() = default;

    void clear_registry();

    void set_registry(const std::shared_ptr<YoRegistry>& registry);

    std::shared_ptr<YoRegistry>& get_registry();

    const std::string& get_name();

    void reset_full_name();
    
protected:
    const std::string name_;
    std::shared_ptr<YoRegistry> registry_;
    std::string full_name_;
};