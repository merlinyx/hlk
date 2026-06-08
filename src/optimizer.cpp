#include "optimizer.h"

#include <iostream>

using namespace std;
using namespace z3;

namespace hlk {

    IntProp::IntProp(context& context, string id) :
            var(context.int_const(id.c_str()))
    {
        val = 0;
        is_fixed = false;
    }
    bool IntProp::set(int value, bool fix)
    {
        bool changed = value != val;
        val = value;
        is_fixed = fix;
        return changed;
    }
    bool IntProp::set_from_model(model& model)
    {
        auto inferred = model.eval(var, true).get_numeral_int();
        bool changed = inferred != val;
        val = inferred;
        return changed;
    }
    void IntProp::unfix()
    {
        is_fixed = false;
    }
    void IntProp::print_info(std::string line_prefix)
    {
        cout << line_prefix << "val = " << val << "  ,  is_fixed = " << is_fixed << endl;
    }
    void IntProp::save(std::ofstream & f)
    {
        f << val << " " << is_fixed << " ";
    }
    void IntProp::load(std::ifstream & f)
    {
        int v;
        f >> val;
        f >> v;
        is_fixed = v;
    }
    BoolProp::BoolProp(context& context, string id) :
            var(context.bool_const(id.c_str()))
    {
        val = false;
        is_fixed = false;
    }
    bool BoolProp::set(bool value, bool fix)
    {
        bool changed = value != val;
        val = value;
        is_fixed = fix;
        return changed;
    }
    void BoolProp::unfix()
    {
        is_fixed = false;
    }
    bool BoolProp::set_from_model(model& model)
    {
        bool changed = false;
        auto inferred = model.eval(var).bool_value();
        switch (inferred) {
            case Z3_L_TRUE:
                changed = !val;
                val = true;
                break;
            case Z3_L_FALSE:
                changed = val;
                val = false;
                break;
            default:
                break;
        }
        return changed;
    }

    void BoolProp::print_info(std::string line_prefix)
    {
        cout << line_prefix << "val = " << val << "  ,  is_fixed = " << is_fixed << endl;
    }

    void BoolProp::save(std::ofstream & f)
    {
        f << val << " " << is_fixed << " ";
    }

    void BoolProp::load(std::ifstream & f)
    {
        int v;
        f >> v;
        val = v;
        f >> v;
        is_fixed = v;
    }

    Optimizer::Optimizer() : solver(context)
    {
        // Nothing to do
    }

    Optimizer::Result Optimizer::solve()
    {
        solver.push();
        add_property_constraints();
        bool is_sat = sat == solver.check();
        Result result;
        if (is_sat) {
            result.set_model(solver.get_model());
        }
        else {
            result.set_unsat_core(solver.unsat_core());
        }
        solver.pop();
        return result;
    }

    Optimizer::Result Optimizer::minimize(z3::expr objective, unsigned int timeout, Strategy strategy) {
        switch (strategy) {
            case BINARY_SEARCH:
                return minimize_bs(objective, timeout);
            case LINEAR_DECREASING:
                return minimize_dec(objective, timeout);
            default:
                return minimize_inc(objective, timeout);
        }
    }

    Optimizer::Result Optimizer::minimize_bs(z3::expr objective, unsigned int timeout)
    {
        solver.push();

        add_property_constraints();

        cout << "Checking SAT" << endl;
        auto is_sat = sat == solver.check();
        cout << "SAT checked." << endl;
        Result result;

        if (is_sat) {
            result.set_model(solver.get_model());

            int max_objective = result.result_model->eval(objective).get_numeral_int();
            cout << "Best so far = " << max_objective << endl;

            if (timeout != -1U) {
                z3::params p(context);
                p.set(":timeout", timeout * 1000u);
                solver.set(p);
            }

            int above = max_objective;
            int below = 0;
            int upper_bound = 0;
            int count = 0;

            while (above >= below) {
                ++count;
                upper_bound = (above + below) / 2;
                cout << "Trying for " << upper_bound << endl;

                solver.push();
                solver.add(objective <= upper_bound);

                try {
                    is_sat = sat == solver.check();
                }
                catch (z3::exception ex) {
                    is_sat = false;
                }

                if (is_sat) {
                    result.set_model(solver.get_model());
                    above = upper_bound - 1;
                }
                else {
                    below = upper_bound + 1;
                }
                solver.pop();
            }
            cout << "Success after " << count << " trials\n";
        }

        solver.pop();
        return result;
    }

    Optimizer::Result Optimizer::minimize_inc(z3::expr objective, unsigned int timeout)
    {
        solver.push();

        add_property_constraints();

        cout << "Checking SAT" << endl;
        auto is_sat = sat == solver.check();
        cout << "SAT checked." << endl;
        Result result;

        if (is_sat) {
            result.set_model(solver.get_model());

            int max_objective = result.result_model->eval(objective).get_numeral_int();
            cout << "Best so far = " << max_objective << endl;
            for (int i = 0; i < max_objective; ++i) {
                cout << "Trying for " << i << " : " << (double)i / max_objective << "%" << endl;
                solver.push();
                solver.add(objective <= i);
                auto constrained_sat = solver.check();

                if (sat == constrained_sat) {
                    result.set_model(solver.get_model());
                    solver.pop();
                    break;
                }
                solver.pop();
            }
        }

        solver.pop();
        return result;
    }

    Optimizer::Result Optimizer::minimize_dec(z3::expr objective, unsigned int timeout)
    {
        solver.push();

        add_property_constraints();

        cout << "Checking SAT" << endl;
        auto is_sat = sat == solver.check();
        cout << "SAT checked." << endl;
        Result result;

        if (is_sat) {
            result.set_model(solver.get_model());

            int max_objective = result.result_model->eval(objective).get_numeral_int();
            cout << "Best so far = " << max_objective << endl;

            z3::params p(context);
            p.set(":timeout", timeout * 1000u);
            solver.set(p);

            int upper_bound = max_objective - 1;
            while (is_sat) {
                cout << "Trying for " << upper_bound << endl;
                solver.push();
                solver.add(objective <= upper_bound);

                try {
                    is_sat = sat == solver.check();
                }
                catch (z3::exception ex) {
                    is_sat = false;
                }


                if (is_sat) {
                    result.set_model(solver.get_model());
                }
                solver.pop();
                --upper_bound;
            }

        }

        solver.pop();
        return result;
    }

    bool Optimizer::update_all_props(model model)
    {
        bool updated = false;
        for (auto& bp : bool_properties) {
            if (!bp->is_fixed) {
                updated |= bp->set_from_model(model);
            }
        }

        for (auto& ip : int_properties) {
            if (!ip->is_fixed) {
                updated |= ip->set_from_model(model);
            }
        }

        return updated;
    }


    void Optimizer::add_constraint(z3::expr constraint, std::string name)
    {
        solver.add(constraint, name.c_str());
    }

    void Optimizer::add_constraint(z3::expr constraint)
    {
        solver.add(constraint);
    }

    void Optimizer::push()
    {
        solver.push();
    }

    void Optimizer::pop()
    {
        solver.pop();
    }

    std::shared_ptr<BoolProp> Optimizer::get_bool_prop(std::string id)
    {
        bool_properties.push_back(make_shared<BoolProp>(context, id));
        return bool_properties.back();
    }

    std::shared_ptr<IntProp> Optimizer::get_int_prop(std::string id)
    {
        int_properties.push_back(make_shared<IntProp>(context, id));
        return int_properties.back();
    }

    void Optimizer::print_info()
    {
        cout << solver.to_smt2() << endl;
        cout << solver.unsat_core() << endl;
    }

    void Optimizer::clear()
    {
        bool_properties.clear();
        int_properties.clear();
        solver = z3::solver(context);
    }

    void Optimizer::add_property_constraints()
    {
        for (auto& bp : bool_properties) {
            if (bp->is_fixed) {
                if (bp->val) {
                    solver.add(bp->var);
                }
                else {
                    solver.add(!(bp->var));
                }
            }
        }
        for (auto& ip : int_properties) {
            if (ip->is_fixed) {
                solver.add(ip->var == ip->val);
            }
        }
    }

    void Optimizer::Result::set_model(z3::model m)
    {
        result_model = make_unique<model>(m);
        has_result = true;
    }

    void Optimizer::Result::set_unsat_core(z3::expr_vector u)
    {
        unsat_core = make_unique<expr_vector>(u);
        has_unsat_core = true;
    }

};
