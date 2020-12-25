//
// Created by han on 2020/12/25.
//

#include "Jumping.h"

Jumping::Jumping(int var_num_per_node, int node_num, double dt):
                var_num_per_node_(var_num_per_node),
                node_num_(node_num),
                dt_(dt){
                    program_ = std::make_shared<drake::solvers::MathematicalProgram>();
                    var_sol_ = std::make_unique<Eigen::VectorXd>(node_num_);
                }

void Jumping::build()
{
    //1. construct per node
    for (int i = 0; i < node_num_; ++i) {
        node_list_.emplace_back(std::make_unique<OptNode>(std::to_string(i), var_num_per_node_, program_));
        node_list_[i]->build();
    }
    for (int i = 0; i < (node_num_-1); ++i) {
        // x dotx
        direct_collocation_constraints_.push_back(
                program_->AddConstraint((node_list_[i+1]->decision_var_ptr_[6]-node_list_[i]->decision_var_ptr_[6])-
                0.5*dt_*(node_list_[i+1]->decision_var_ptr_[7]+node_list_[i]->decision_var_ptr_[7]) == 0).evaluator().get());
        // dotx ddotx
        direct_collocation_constraints_.push_back(
                program_->AddConstraint((node_list_[i+1]->decision_var_ptr_[7]-node_list_[i]->decision_var_ptr_[7])-
                                        0.5*dt_*(node_list_[i+1]->decision_var_ptr_[8]+node_list_[i]->decision_var_ptr_[8]) == 0).evaluator().get());
        // s dots
        direct_collocation_constraints_.push_back(
                program_->AddConstraint((node_list_[i+1]->decision_var_ptr_[0]-node_list_[i]->decision_var_ptr_[0])-
                                        0.5*dt_*(node_list_[i+1]->decision_var_ptr_[1]+node_list_[i]->decision_var_ptr_[1]) == 0).evaluator().get());
        // dots ddots
        direct_collocation_constraints_.push_back(
                program_->AddConstraint((node_list_[i+1]->decision_var_ptr_[1]-node_list_[i]->decision_var_ptr_[1])-
                                        0.5*dt_*(node_list_[i+1]->decision_var_ptr_[2]+node_list_[i]->decision_var_ptr_[2]) == 0).evaluator().get());
        // l dotl
        direct_collocation_constraints_.push_back(
                program_->AddConstraint((node_list_[i+1]->decision_var_ptr_[3]-node_list_[i]->decision_var_ptr_[3])-
                                        0.5*dt_*(node_list_[i+1]->decision_var_ptr_[4]+node_list_[i]->decision_var_ptr_[4]) == 0).evaluator().get());
        // dotl ddotl
        direct_collocation_constraints_.push_back(
                program_->AddConstraint((node_list_[i+1]->decision_var_ptr_[4]-node_list_[i]->decision_var_ptr_[4])-
                                        0.5*dt_*(node_list_[i+1]->decision_var_ptr_[5]+node_list_[i]->decision_var_ptr_[5]) == 0).evaluator().get());
    }

    kinematics_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(1) == 0).evaluator().get());

    kinematics_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(2) == 0).evaluator().get());

    kinematics_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(4) == 0).evaluator().get());

    kinematics_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(5) == 0).evaluator().get());

    kinematics_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(7) == 0).evaluator().get());
    kinematics_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(8) == 0).evaluator().get());

    kinematics_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[node_num_-1]->decision_var_ptr_(3) == 1).evaluator().get());

    kinematics_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[node_num_-1]->decision_var_ptr_(4) == 2).evaluator().get());

    drake::symbolic::Expression Cost;

    for (int i = 0; i < node_num_; ++i) {
        Cost += drake::symbolic::pow(node_list_[i]->decision_var_ptr_(5), 2);
    }
    program_->AddCost(Cost);

}

void Jumping::print_var(const drake::solvers::MathematicalProgramResult& result)
{
    for (int i = 0; i < node_num_; ++i) {
        node_list_[i]->print_var(result);
    }
}