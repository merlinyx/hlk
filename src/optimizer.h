#pragma once

#include "z3++.h"
#include "z3_api.h"
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <fstream>

namespace hlk {

    class BoolProp {
    public:
        BoolProp(z3::context& context, std::string id);
        bool set(bool value, bool fix = true);
        void unfix();
        bool set_from_model(z3::model& model);
        bool val;
        bool is_fixed;
        z3::expr var;
        void print_info(std::string line_prefix);
        void save(std::ofstream& f);
        void load(std::ifstream& f);

        operator bool() const {return val;}
        operator z3::expr() const {return var;}

        friend z3::expr operator!(const BoolProp& v) { return !v.var; }

        friend z3::expr operator&&(const BoolProp& l, const BoolProp& r) { return l.var && r.var; }
        friend z3::expr operator||(const BoolProp& l, const BoolProp& r) { return l.var || r.var; }
        friend z3::expr operator==(const BoolProp& l, const BoolProp& r) { return l.var == r.var; }
        friend z3::expr operator!=(const BoolProp& l, const BoolProp& r) { return l.var != r.var; }

        friend z3::expr operator&&(const z3::expr& l, const BoolProp& r) { return l && r.var; }
        friend z3::expr operator||(const z3::expr& l, const BoolProp& r) { return l || r.var; }
        friend z3::expr operator==(const z3::expr& l, const BoolProp& r) { return l == r.var; }
        friend z3::expr operator!=(const z3::expr& l, const BoolProp& r) { return l != r.var; }

        friend z3::expr operator&&(const BoolProp& l, const z3::expr& r) { return l.var && r; }
        friend z3::expr operator||(const BoolProp& l, const z3::expr& r) { return l.var || r; }
        friend z3::expr operator==(const BoolProp& l, const z3::expr& r) { return l.var == r; }
        friend z3::expr operator!=(const BoolProp& l, const z3::expr& r) { return l.var != r; }

        friend z3::expr operator&&(const std::shared_ptr<BoolProp>& l, const std::shared_ptr<BoolProp>& r) {
            return l->var == r->var;
        }
        friend z3::expr operator||(const std::shared_ptr<BoolProp>& l, const std::shared_ptr<BoolProp>& r) {
            return l->var == r->var;
        }
        friend z3::expr operator==(const std::shared_ptr<BoolProp>& l, const std::shared_ptr<BoolProp>& r) {
            return l->var == r->var;
        }
        friend z3::expr operator!=(const std::shared_ptr<BoolProp>& l, const std::shared_ptr<BoolProp>& r) {
            return l->var != r->var;
        }

        friend z3::expr operator&&(const z3::expr& l, const std::shared_ptr<BoolProp>& r) {
            return l == r->var;
        }
        friend z3::expr operator||(const z3::expr& l, const std::shared_ptr<BoolProp>& r) {
            return l == r->var;
        }
        friend z3::expr operator==(const z3::expr& l, const std::shared_ptr<BoolProp>& r) {
            return l == r->var;
        }
        friend z3::expr operator!=(const z3::expr& l, const std::shared_ptr<BoolProp>& r) {
            return l != r->var;
        }

        friend z3::expr operator&&(const std::shared_ptr<BoolProp>& l, const z3::expr& r) {
            return l->var == r;
        }
        friend z3::expr operator||(const std::shared_ptr<BoolProp>& l, const z3::expr& r) {
            return l->var == r;
        }
        friend z3::expr operator==(const std::shared_ptr<BoolProp>& l, const z3::expr& r) {
            return l->var == r;
        }
        friend z3::expr operator!=(const std::shared_ptr<BoolProp>& l, const z3::expr& r) {
            return l->var != r;
        }

    };

    class IntProp {
    public:
        IntProp(z3::context& context, std::string id);
        bool set(int value, bool fix = true);
        bool set_from_model(z3::model& model);
        void unfix();
        int val;
        bool is_fixed;
        z3::expr var;
        void print_info(std::string line_prefix);
        void save(std::ofstream& f);
        void load(std::ifstream& f);

        operator int() const {return val;}
        operator z3::expr() const {return var;}

        friend z3::expr operator+(const IntProp& l, const IntProp& r) { return l.var + r.var; }
        friend z3::expr operator-(const IntProp& l, const IntProp& r) { return l.var - r.var; }
        friend z3::expr operator*(const IntProp& l, const IntProp& r) { return l.var * r.var; }
        friend z3::expr operator<(const IntProp& l, const IntProp& r) { return l.var < r.var; }
        friend z3::expr operator>(const IntProp& l, const IntProp& r) { return l.var > r.var; }
        friend z3::expr operator<=(const IntProp& l, const IntProp& r) { return l.var <= r.var; }
        friend z3::expr operator>=(const IntProp& l, const IntProp& r) { return l.var >= r.var; }
        friend z3::expr operator==(const IntProp& l, const IntProp& r) { return l.var == r.var; }
        friend z3::expr operator!=(const IntProp& l, const IntProp& r) { return l.var != r.var; }

        friend z3::expr operator+(const z3::expr& l, const IntProp& r) { return l + r.var; }
        friend z3::expr operator-(const z3::expr& l, const IntProp& r) { return l - r.var; }
        friend z3::expr operator*(const z3::expr& l, const IntProp& r) { return l * r.var; }
        friend z3::expr operator<(const z3::expr& l, const IntProp& r) { return l < r.var; }
        friend z3::expr operator>(const z3::expr& l, const IntProp& r) { return l > r.var; }
        friend z3::expr operator<=(const z3::expr& l, const IntProp& r) { return l <= r.var; }
        friend z3::expr operator>=(const z3::expr& l, const IntProp& r) { return l >= r.var; }
        friend z3::expr operator==(const z3::expr& l, const IntProp& r) { return l == r.var; }
        friend z3::expr operator!=(const z3::expr& l, const IntProp& r) { return l != r.var; }

        friend z3::expr operator+(const IntProp& l, const z3::expr& r) { return l.var + r; }
        friend z3::expr operator-(const IntProp& l, const z3::expr& r) { return l.var - r; }
        friend z3::expr operator*(const IntProp& l, const z3::expr& r) { return l.var * r; }
        friend z3::expr operator<(const IntProp& l, const z3::expr& r) { return l.var < r; }
        friend z3::expr operator>(const IntProp& l, const z3::expr& r) { return l.var > r; }
        friend z3::expr operator<=(const IntProp& l, const z3::expr& r) { return l.var <= r; }
        friend z3::expr operator>=(const IntProp& l, const z3::expr& r) { return l.var >= r; }
        friend z3::expr operator==(const IntProp& l, const z3::expr& r) { return l.var == r; }
        friend z3::expr operator!=(const IntProp& l, const z3::expr& r) { return l.var != r; }

        friend z3::expr operator+(const std::shared_ptr<IntProp>& l, const std::shared_ptr<IntProp>& r) { return l->var + r->var; }
        friend z3::expr operator-(const std::shared_ptr<IntProp>& l, const std::shared_ptr<IntProp>& r) { return l->var - r->var; }
        friend z3::expr operator*(const std::shared_ptr<IntProp>& l, const std::shared_ptr<IntProp>& r) { return l->var * r->var; }
        friend z3::expr operator<(const std::shared_ptr<IntProp>& l, const std::shared_ptr<IntProp>& r) { return l->var < r->var; }
        friend z3::expr operator>(const std::shared_ptr<IntProp>& l, const std::shared_ptr<IntProp>& r) { return l->var > r->var; }
        friend z3::expr operator<=(const std::shared_ptr<IntProp>& l, const std::shared_ptr<IntProp>& r) { return l->var <= r->var; }
        friend z3::expr operator>=(const std::shared_ptr<IntProp>& l, const std::shared_ptr<IntProp>& r) { return l->var >= r->var; }
        friend z3::expr operator==(const std::shared_ptr<IntProp>& l, const std::shared_ptr<IntProp>& r) { return l->var == r->var; }
        friend z3::expr operator!=(const std::shared_ptr<IntProp>& l, const std::shared_ptr<IntProp>& r) { return l->var != r->var; }

        friend z3::expr operator+(const z3::expr& l, const std::shared_ptr<IntProp>& r) { return l + r->var; }
        friend z3::expr operator-(const z3::expr& l, const std::shared_ptr<IntProp>& r) { return l - r->var; }
        friend z3::expr operator*(const z3::expr& l, const std::shared_ptr<IntProp>& r) { return l * r->var; }
        friend z3::expr operator<(const z3::expr& l, const std::shared_ptr<IntProp>& r) { return l < r->var; }
        friend z3::expr operator>(const z3::expr& l, const std::shared_ptr<IntProp>& r) { return l > r->var; }
        friend z3::expr operator<=(const z3::expr& l, const std::shared_ptr<IntProp>& r) { return l <= r->var; }
        friend z3::expr operator>=(const z3::expr& l, const std::shared_ptr<IntProp>& r) { return l >= r->var; }
        friend z3::expr operator==(const z3::expr& l, const std::shared_ptr<IntProp>& r) { return l == r->var; }
        friend z3::expr operator!=(const z3::expr& l, const std::shared_ptr<IntProp>& r) { return l != r->var; }

        friend z3::expr operator+(const std::shared_ptr<IntProp>& l, const z3::expr& r) { return l->var + r; }
        friend z3::expr operator-(const std::shared_ptr<IntProp>& l, const z3::expr& r) { return l->var - r; }
        friend z3::expr operator*(const std::shared_ptr<IntProp>& l, const z3::expr& r) { return l->var * r; }
        friend z3::expr operator<(const std::shared_ptr<IntProp>& l, const z3::expr& r) { return l->var < r; }
        friend z3::expr operator>(const std::shared_ptr<IntProp>& l, const z3::expr& r) { return l->var > r; }
        friend z3::expr operator<=(const std::shared_ptr<IntProp>& l, const z3::expr& r) { return l->var <= r; }
        friend z3::expr operator>=(const std::shared_ptr<IntProp>& l, const z3::expr& r) { return l->var >= r; }
        friend z3::expr operator==(const std::shared_ptr<IntProp>& l, const z3::expr& r) { return l->var == r; }
        friend z3::expr operator!=(const std::shared_ptr<IntProp>& l, const z3::expr& r) { return l->var != r; }

        friend z3::expr operator+(int l, const std::shared_ptr<IntProp>& r) { return l + r->var; }
        friend z3::expr operator-(int l, const std::shared_ptr<IntProp>& r) { return l - r->var; }
        friend z3::expr operator*(int l, const std::shared_ptr<IntProp>& r) { return l * r->var; }
        friend z3::expr operator<(int l, const std::shared_ptr<IntProp>& r) { return l < r->var; }
        friend z3::expr operator>(int l, const std::shared_ptr<IntProp>& r) { return l > r->var; }
        friend z3::expr operator<=(int l, const std::shared_ptr<IntProp>& r) { return l <= r->var; }
        friend z3::expr operator>=(int l, const std::shared_ptr<IntProp>& r) { return l >= r->var; }
        friend z3::expr operator==(int l, const std::shared_ptr<IntProp>& r) { return l == r->var; }
        friend z3::expr operator!=(int l, const std::shared_ptr<IntProp>& r) { return l != r->var; }

        friend z3::expr operator+(const std::shared_ptr<IntProp>& l, int r) { return l->var + r; }
        friend z3::expr operator-(const std::shared_ptr<IntProp>& l, int r) { return l->var - r; }
        friend z3::expr operator*(const std::shared_ptr<IntProp>& l, int r) { return l->var * r; }
        friend z3::expr operator<(const std::shared_ptr<IntProp>& l, int r) { return l->var < r; }
        friend z3::expr operator>(const std::shared_ptr<IntProp>& l, int r) { return l->var > r; }
        friend z3::expr operator<=(const std::shared_ptr<IntProp>& l, int r) { return l->var <= r; }
        friend z3::expr operator>=(const std::shared_ptr<IntProp>& l, int r) { return l->var >= r; }
        friend z3::expr operator==(const std::shared_ptr<IntProp>& l, int r) { return l->var == r; }
        friend z3::expr operator!=(const std::shared_ptr<IntProp>& l, int r) { return l->var != r; }
    };

    class Optimizer {
    public:

        struct Result {
            void set_model(z3::model m);
            void set_unsat_core(z3::expr_vector u);
            bool has_result = false;
            bool has_unsat_core = false;
            std::unique_ptr<z3::model> result_model;
            std::unique_ptr<z3::expr_vector> unsat_core;
        };

        enum Strategy {
            BINARY_SEARCH,
            LINEAR_INCREASING,
            LINEAR_DECREASING
        };

        Optimizer();
        Result solve();
        Result minimize(z3::expr objective, unsigned int timeout = -1U, Strategy strategy = BINARY_SEARCH);
        bool update_all_props(z3::model model);
        void add_constraint(z3::expr constraint);
        void add_constraint(z3::expr constraint, std::string name);
        void push();
        void pop();
        std::shared_ptr<BoolProp> get_bool_prop(std::string id);
        std::shared_ptr<IntProp> get_int_prop(std::string id);
        z3::context context;
        void print_info();
        void clear();
    //private:
        Result minimize_bs(z3::expr objective, unsigned int timeout = -1U);
        Result minimize_inc(z3::expr objective, unsigned int timeout = -1U);
        Result minimize_dec(z3::expr objective, unsigned int timeout = -1U); // timeout in seconds
        void add_property_constraints();
        std::vector<std::shared_ptr<BoolProp>> bool_properties;
        std::vector<std::shared_ptr<IntProp>> int_properties;

        z3::solver solver;
    };

};
