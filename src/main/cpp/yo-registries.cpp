#include "yo-registries.hpp"
#include "yo-variables.hpp"


namespace ihmc
{

YoRegistry::YoRegistry(const std::string& name) : name_(name) {}

const std::string& YoRegistry::get_name() const 
{
    return name_;
}

void YoRegistry::detatch_from_parent()
{
    if (parent_ == nullptr)
        return;
    
    parent_->remove_child(this->shared_from_this());
}

void YoRegistry::set_parent_namespace(std::shared_ptr<YoNamespace> parent_namespace)
{
    if (parent_namespace == nullptr)
    {
        namespace_ = std::make_shared<YoNamespace>(name_);
    }
    else
    {
        namespace_ = std::make_shared<YoNamespace>(parent_namespace->append(name_));
    }
    namespace_->check_sanity();

    for (auto& child : children_)
    {
        child->set_parent_namespace(namespace_);
    }

    for (auto& variable : variables_)
    {
        variable->reset_full_name();
    }
}

const std::shared_ptr<YoNamespace>& YoRegistry::get_namespace() const
{
    return namespace_;
}

bool YoRegistry::has_variable(const std::string& name) const
{
    return name_to_variable_map_.count(name) > 0;
}


void YoRegistry::notify_listeners(std::shared_ptr<YoRegistry> target_parent_registry, std::shared_ptr<YoRegistry> registry, std::shared_ptr<YoVariable> variable, const YoRegistry::ChangeType change_type)
{
    if (!changed_listeners_.empty())
    {
        auto change = RegistryChange(target_parent_registry, registry, variable, change_type, this->shared_from_this());
        auto change_ptr = std::make_shared<RegistryChange>(change);
        for (auto& listener : changed_listeners_)
            listener->changed(change_ptr);
    }
}

void YoRegistry::remove_variable(std::shared_ptr<YoVariable> variable)
{
    if (!has_variable(variable->get_name()))
        return;

    std::string var_name = variable->get_name();
    std::transform(var_name.begin(), var_name.end(), var_name.begin(), ::tolower);

    if (!removal_allowed())
        throw std::runtime_error("Cannot remove variables from this registry: " + namespace_->get_name());

    variable->clear_registry();
    // remove from the lists
    variables_.erase(std::remove(variables_.begin(), variables_.end(), variable), variables_.end());
    name_to_variable_map_.erase(var_name);

    std::shared_ptr<YoRegistry> nullPtr;
    notify_listeners(this->shared_from_this(), nullPtr, variable, ChangeType::VARIABLE_REMOVED);
}

void YoRegistry::add_variable(std::shared_ptr<YoVariable> variable)
{
    if (!addition_allowed())
        throw std::runtime_error("Cannot add variables to this registry: " + namespace_->get_name());

    if (has_variable(variable->get_name()))
    {
        return;
    }

    std::string var_name = to_lowercase_name(variable->get_name());

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

void YoRegistry::add_child(std::shared_ptr<YoRegistry> child, bool notify_listeners)
{
    if (child == nullptr)
        return;

    if (child.get() == this)
        throw std::runtime_error("Cannot add a registry as a child of itself, registry: " + namespace_->get_name());

    if (!addition_allowed())
        throw std::runtime_error("Cannot add children to this registry: " + namespace_->get_name());

    if (name_to_child_map_.count(child->get_name()))
        return;

    // Make sure no children with this name already.
    std::string child_name = to_lowercase_name(child->get_name());
    if (name_to_child_map_.count(child_name))
    {
        throw std::runtime_error("Name collision for new child: " + child_name + ". Parent name space = " + namespace_->get_name());
    }

    child->detatch_from_parent();
    child->parent_ = this->shared_from_this();
    child->set_parent_namespace(namespace_);

    if (child->get_restriction_level() < restriction_level_)
    {
        child->set_restriction_level(restriction_level_);
    }

    children_.push_back(child);
    name_to_child_map_[child_name] = child;

    if (notify_listeners)
    {
        std::shared_ptr<YoVariable> nullVarPtr;
        this->notify_listeners(this->shared_from_this(), child, nullVarPtr, ChangeType::REGISTRY_ADDED);
    }
}

void YoRegistry::remove_child(std::shared_ptr<YoRegistry> child)
{
    if (child == nullptr || child.get() == this)
        return;

    if (!removal_allowed())
        throw std::runtime_error("Cannot remove children from this registry: " + namespace_->get_name());

    if (!name_to_child_map_.count(child->get_name()))
        return;

    std::string child_name = to_lowercase_name(child->get_name());

    child->parent_ = nullptr;
    child->set_parent_namespace(nullptr);

    // remove from the lists
    children_.erase(std::remove(children_.begin(), children_.end(), child), children_.end());
    name_to_child_map_.erase(child_name);

    std::shared_ptr<YoVariable> nullVarPtr;
    this->notify_listeners(this->shared_from_this(), child, nullVarPtr, ChangeType::REGISTRY_REMOVED);
}

void YoRegistry::add_listener(std::shared_ptr<YoRegistryChangedListener> listener)
{
    if (listener == nullptr)
        this->changed_listeners_.clear();
    this->changed_listeners_.push_back(listener);
}

bool YoRegistry::remove_listener(std::shared_ptr<YoRegistryChangedListener> listener)
{
    if (listener == nullptr)
        return false;

    auto old_end = changed_listeners_.end();
    auto new_end = std::remove(changed_listeners_.begin(), changed_listeners_.end(), listener);
    changed_listeners_.erase(new_end, changed_listeners_.end());

    // if the old end isn't equal to the new end, the size changed, and we removed the value.
    return old_end != new_end;
}


std::shared_ptr<YoVariable> YoRegistry::get_variable(const std::string& name) const
{
    auto it = name_to_variable_map_.find(to_lowercase_name(name));
    if (it != name_to_variable_map_.end())
    {
        return it->second;
    }
    return nullptr;
}

const std::string YoRegistry::to_lowercase_name(const std::string& name) const
{ 
    std::string var_name = name;
    std::transform(var_name.begin(), var_name.end(), var_name.begin(), ::tolower);

    return var_name;
}

void YoRegistry::set_restriction_level(YoRegistry::YoRegistryRestrictionLevel restriction_level)
{
    restriction_level_ = restriction_level;
}

const YoRegistry::YoRegistryRestrictionLevel YoRegistry::get_restriction_level() const
{
    return restriction_level_;
}

bool YoRegistry::operator==(const YoRegistry& other) const
{
    if (this == &other)
        return true;

    if (this->get_namespace() != other.get_namespace())
        return false;

    if (this->variables_.size() != other.variables_.size())
        return false;

    for (auto& variable : this->variables_)
    {
        if (!other.name_to_variable_map_.count(to_lowercase_name(variable->get_name())))
            return false;
    }

    if (this->children_.size() != other.children_.size())
        return false;

    for (auto& child : this->children_)
    {
        if (!other.name_to_child_map_.count(to_lowercase_name(child->get_name())))
            return false;
    }

    return true;
}
} // namespace ihmc