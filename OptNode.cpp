//
// Created by han on 2020/12/25.
//

#include "OptNode.h"

OptNode::OptNode(std::string name, int var_size, std::shared_ptr<drake::solvers::MathematicalProgram> program)
{
    var_name_ = name;
    var_size_ = var_size;
    program_ = program;
    decision_var_ptr_ = program_->NewContinuousVariables(var_size_, var_name_);
    var_sol_ = std::make_unique<Eigen::VectorXd>(var_size);
    var_sol_->setZero();
}

void OptNode::build()
{
    // decision order S L XW

    // spring force cal
    drake::symbolic::Expression spring_force = -34634.77200571 + decision_var_ptr_(3)*167246.05400647+
            decision_var_ptr_(3)*decision_var_ptr_(3)*-200237.85111728+decision_var_ptr_(3)*
            decision_var_ptr_(3)*decision_var_ptr_(3)*decision_var_ptr_(3)*91464.71714233;

    dynamic_constraints_.push_back(
            program_->AddConstraint(decision_var_ptr_(8)-spring_force*decision_var_ptr_(0)/31+9.81
                                    == 0).evaluator().get());

    dynamic_constraints_.push_back(
            program_->AddConstraint(decision_var_ptr_(2)-decision_var_ptr_(5)+decision_var_ptr_(8)
            == 0).evaluator().get());

    dynamic_constraints_.push_back(
            program_->AddConstraint(decision_var_ptr_(0) + decision_var_ptr_(6) -
            decision_var_ptr_(3) == 0).evaluator().get()
            );

    dynamic_constraints_.push_back(
            program_->AddConstraint(decision_var_ptr_(1) + decision_var_ptr_(7) -
                                    decision_var_ptr_(4) == 0).evaluator().get()
    );

    limit_constraints_.push_back(
            program_->AddLinearConstraint(decision_var_ptr_(0), 0, 0.2).evaluator().get());

    limit_constraints_.push_back(
            program_->AddLinearConstraint(decision_var_ptr_(3), 0.65, 1.1).evaluator().get());

    limit_constraints_.push_back(
            program_->AddLinearConstraint(decision_var_ptr_(6), 0.65, 1.1).evaluator().get());


}

void OptNode::print_var(const drake::solvers::MathematicalProgramResult& result)
{
    *var_sol_ = result.GetSolution(decision_var_ptr_);
    std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<std::endl;
    std::cout<<(*var_sol_)(0)<<std::endl;

}
