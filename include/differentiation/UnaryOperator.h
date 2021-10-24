#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class UnaryOperator : public Expression {
public:
    UnaryOperator() = delete;

    explicit UnaryOperator(ExprPtr operand) : operand(std::move(operand)) {}

    [[nodiscard]] double evaluate(const std::map<std::string, double> &variables) const override;

    [[nodiscard]] ExprPtr diff(const std::string &id) const override;

    [[nodiscard]] ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const override;

    [[nodiscard]] ExprPtr simplify() const override;

    [[nodiscard]] std::string toStr() const override;

    [[nodiscard]] int nodeCount() const override;

protected:
    [[nodiscard]] virtual double call(const double &operand) const = 0;

    [[nodiscard]] virtual ExprPtr call(const ExprPtr &operand) const = 0;

    [[nodiscard]] virtual ExprPtr derivative(const ExprPtr &expr) const = 0;

    [[nodiscard]] virtual std::string toStrWrapper(const std::string &operandString) const = 0;

private:
    ExprPtr operand;
};
