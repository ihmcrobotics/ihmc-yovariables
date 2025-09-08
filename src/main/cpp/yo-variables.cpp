#include "yo-variables.hpp"

namespace ihmc
{
// Constructor implementation
YoVariable::YoVariable(const std::string& name, std::shared_ptr<YoRegistry> registry)  : name_(name), registry_(registry) {}

void YoVariable::reset_full_name()
{
    full_name_ = nullptr;
}

void YoVariable::clear_registry()
{
    auto old_registry = registry_;
    registry_ = nullptr;

    if (old_registry != nullptr && old_registry->has_variable(this->get_name()))
    {
        old_registry->remove_variable(this->shared_from_this());
    }
    reset_full_name();
}

void YoVariable::set_registry(std::shared_ptr<YoRegistry> registry)
{
    if (registry == registry_)
        return;

    clear_registry();

    if (registry != nullptr)
    {
        std::shared_ptr<YoVariable> existing_variable = registry->get_variable(get_name());
        if (existing_variable != nullptr && existing_variable.get() != this)
            throw std::invalid_argument("A variable with the name \"" + this->get_name() + "\" already exists in the target registry.");

        registry->add_variable(this->shared_from_this());
    }
    
    registry_ = registry;
    reset_full_name();
}

std::shared_ptr<YoRegistry>& YoVariable::get_registry() 
{ 
    return registry_; 
}

const std::string& YoVariable::get_name() const
{ 
    return name_;
}

bool YoVariable::operator==(const YoVariable& other) const
{
    if (this == &other)
        return true;

    return this->get_name() != other.get_name();
}
} // namespace ihmc