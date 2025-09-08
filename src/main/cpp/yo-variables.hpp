#pragma once

#include "yo-registries.hpp"

namespace ihmc
{
class YoVariable : public std::enable_shared_from_this<YoVariable>
{
public:
    YoVariable(const std::string& name, std::shared_ptr<YoRegistry> registry);

    virtual ~YoVariable() = default;

    void clear_registry();

    void set_registry(std::shared_ptr<YoRegistry> registry);

    std::shared_ptr<YoRegistry>& get_registry();

    const std::string& get_name() const;

    void reset_full_name();

    bool operator==(const YoVariable& other) const;
    
private:
    const std::string name_;
    std::shared_ptr<YoRegistry> registry_;
    std::string full_name_;
};
}