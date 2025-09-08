#pragma once

#include <string>
#include <vector>
#include <numeric>
#include <sstream>
#include <memory>
#include <unordered_map>
#include "ihmc-yovariables.hpp"
#include "yo-tools.hpp"
#include "yo-registry-change-listener.hpp"

namespace ihmc
{
class YoRegistry : public std::enable_shared_from_this<YoRegistry>
{
public:
    explicit YoRegistry(const std::string& name);

    const std::string& get_name() const;

    const std::shared_ptr<YoNamespace>& get_namespace() const;

    void detatch_from_parent();

    void set_parent_namespace(std::shared_ptr<YoNamespace> parent_namespace);

    bool has_variable(const std::string& name) const;

    void add_variable(std::shared_ptr<YoVariable> variable);

    void remove_variable(std::shared_ptr<YoVariable> variable);

    inline void add_child(std::shared_ptr<YoRegistry> child)
    {
        add_child(child, true);
    }

    void add_child(std::shared_ptr<YoRegistry> child, bool notify_listeners);

    void remove_child(std::shared_ptr<YoRegistry> child);

    std::shared_ptr<YoRegistry> get_child(const std::string& name) const;

    std::shared_ptr<YoVariable> get_variable(const std::string& name) const;

    void add_listener(std::shared_ptr<YoRegistryChangedListener> listener);

    bool remove_listener(std::shared_ptr<YoRegistryChangedListener> listener);

    bool operator==(const YoRegistry& other) const;

protected: 
    std::shared_ptr<YoRegistry> parent_ = nullptr;

    enum ChangeType
    {
        REGISTRY_ADDED,
        REGISTRY_REMOVED,
        VARIABLE_ADDED,
        VARIABLE_REMOVED,
        CLEARED
    };

private:

    enum YoRegistryRestrictionLevel
    {
        FULLY_MUTABLE,
        RESTRICTED,
        IMMUTABLE
    };

    class RegistryChange : public YoRegistryChangedListener::Change
    {
    public:
        RegistryChange(const std::shared_ptr<YoRegistry> target_parent_registry,
                    const std::shared_ptr<YoRegistry> target_registry, 
                    const std::shared_ptr<YoVariable> target_variable,
                    const ChangeType change_type,
                    const std::shared_ptr<YoRegistry> source) : 
                     target_parent_registry_(target_parent_registry), target_registry_(target_registry), target_variable_(target_variable), change_type_(change_type), source_(source) {}

        inline const bool was_registry_added() const { return this->change_type_ == YoRegistry::ChangeType::REGISTRY_ADDED; }

        inline const bool was_registry_removed() const { return this->change_type_ == YoRegistry::ChangeType::REGISTRY_REMOVED; }

        inline const bool was_variable_added() const { return this->change_type_ == YoRegistry::ChangeType::VARIABLE_ADDED; }

        inline const bool was_variable_removed() const { return this->change_type_ == YoRegistry::ChangeType::VARIABLE_REMOVED; }

        inline const bool was_cleared() const { return this->change_type_ == YoRegistry::ChangeType::VARIABLE_REMOVED; }

        inline const std::shared_ptr<YoRegistry> get_source() const { return source_; }

        inline const std::shared_ptr<YoRegistry> get_target_parent_registry() const { return target_parent_registry_; }

        inline const std::shared_ptr<YoRegistry> get_target_registry() const { return target_registry_; }

        inline const std::shared_ptr<YoVariable> get_target_variable() const { return target_variable_; }
    private: 
        std::shared_ptr<YoRegistry> target_parent_registry_;
        std::shared_ptr<YoRegistry> target_registry_;
        std::shared_ptr<YoVariable> target_variable_;
        ChangeType change_type_;
        std::shared_ptr<YoRegistry> source_;
    };


    inline bool removal_allowed() const
    {
        return restriction_level_ == FULLY_MUTABLE;
    }


    inline bool addition_allowed() const
    {
        return restriction_level_ != IMMUTABLE;
    }

    YoRegistryRestrictionLevel restriction_level_;
    std::string name_;
    std::shared_ptr<YoNamespace> namespace_;
    std::vector<std::shared_ptr<YoRegistryChangedListener>> changed_listeners_;
    std::vector<std::shared_ptr<YoVariable>> variables_;
    std::vector<std::shared_ptr<YoRegistry>> children_;
    std::unordered_map<std::string, std::shared_ptr<YoVariable>> name_to_variable_map_;
    std::unordered_map<std::string, std::shared_ptr<YoRegistry>> name_to_child_map_;

    void notify_listeners(std::shared_ptr<YoRegistry> target_parent_registry, 
                          std::shared_ptr<YoRegistry> registry,
                          std::shared_ptr<YoVariable> variable, 
                          const ChangeType change_type);

    const std::string to_lowercase_name(const std::string& name) const;

    void set_restriction_level(YoRegistryRestrictionLevel restriction_level);

    const YoRegistryRestrictionLevel get_restriction_level() const;
};
} // namespace ihmc