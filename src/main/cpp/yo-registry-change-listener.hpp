#pragma once

#include "ihmc-yovariables.hpp"

namespace ihmc
{
class YoRegistryChangedListener 
{
public: 

    class Change
    {
    public:
        virtual const bool was_registry_added() const = 0;

        virtual const bool was_registry_removed() const = 0;

        virtual const bool was_variable_added() const = 0;

        virtual const bool was_variable_removed() const = 0;

        virtual const bool was_cleared() const = 0;

        virtual const std::shared_ptr<YoRegistry> get_source() const = 0;

        virtual const std::shared_ptr<YoRegistry> get_target_registry() const = 0;

        virtual const std::shared_ptr<YoVariable> get_target_variable() const = 0;
    };

    virtual void changed(std::shared_ptr<Change> change) = 0;
};

}