#pragma once

#include "problem/debug.h"
#include "types/string.h"
#include "types/vector.h"

template<class T>
class Matrix
{
  protected:
    Vector<T> data_;
    Size rows_ = 0;
    Size cols_ = 0;

  public:
    // Constructors and destructor
    Matrix(const Size rows, const Size cols, const T& value = T()) :
        data_(rows * cols, value),
        rows_(rows),
        cols_(cols)
    {}
    Matrix() = default;
    Matrix(const Matrix<T>& other) = default;
    Matrix(Matrix<T>&& other) noexcept = default;
    ~Matrix() = default;

    // Assignment
    Matrix<T>& operator=(const Matrix<T>& other) = default;
    Matrix<T>& operator=(Matrix<T>&& other) = default;

    // Modifiers
    void resize_rows(const Size rows, const T& value = T())
    {
        debug_assert(cols_ > 0);
        rows_ = rows;
        data_.clear();
        data_.resize(rows_ * cols_, value);
    }
    void clear_and_resize(const Size rows, const Size cols, const T& value = T())
    {
        data_.clear();
        data_.resize(rows * cols, value);
        rows_ = rows;
        cols_ = cols;
    }
    inline void set(const T& value)
    {
        std::fill(data_.begin(), data_.end(), value);
    }

    // Comparison
    inline bool operator==(const Matrix<T>& other) const { return rows_ == other.rows() && data_ == other.data_; }

    // Getters
    inline auto data() { return data_.data(); }
    inline auto data() const { return data_.data(); }
    inline auto size() const { return data_.size(); }
    inline auto rows() const { return rows_; }
    inline auto cols() const { return cols_; }
    inline const T& operator()(const Size i, const Size j) const
    {
        debug_assert(i < rows());
        debug_assert(j < cols());
        return data_[i * cols_ + j];
    }

    // Setters
    inline T& operator()(const Size i, const Size j)
    {
        debug_assert(i < rows());
        debug_assert(j < cols());
        return data_[i * cols_ + j];
    }

    // Iterators
    inline auto begin() { return data_.begin(); }
    inline auto end() { return data_.end(); }
    inline auto begin(const Size row) { return data_.begin() + (row * cols()); }
    inline auto end(const Size row) { return data_.begin() + ((row + 1) * cols()); }
    inline auto begin(const Size row) const { return data_.begin() + (row * cols()); }
    inline auto end(const Size row) const { return data_.begin() + ((row + 1) * cols()); }
    inline auto cbegin(const Size row) const { return data_.cbegin() + (row * cols()); }
    inline auto cend(const Size row) const { return data_.cbegin() + ((row + 1) * cols()); }

    // Print
    void print() const
    {
        fmt::print("        ");
        for (Size j = 0; j < cols(); ++j)
        {
            fmt::print(" {:>6}", j);
        }
        fmt::print("\n");
        fmt::print("       +");
        for (Size j = 0; j < cols(); ++j)
        {
            fmt::print("-------");
        }
        fmt::print("\n");

        for (Size i = 0; i < rows(); ++i)
        {
            fmt::print("{:>6} |", i);
            for (Size j = 0; j < cols(); ++j)
            {
                if constexpr (std::is_same<T, bool>::value)
                {
                    fmt::print(" {:>6}", static_cast<int>(operator()(i, j)));
                }
                else if constexpr (std::is_same<T, float>::value || std::is_same<T, double>::value)
                {
                    fmt::print(" {:>6.2f}", operator()(i, j));
                }
                else
                {
                    fmt::print(" {:>6}", operator()(i, j));
                }
            }
            fmt::print("\n");
        }
        fmt::print("\n");
        fflush(stdout);
    }
    void print(const String* const row_names, const String* const col_names) const
    {
        fmt::print("            ");
        for (Size j = 0; j < cols(); ++j)
        {
            fmt::print(" {:>10}", col_names[j]);
        }
        fmt::print("\n");
        fmt::print("           +");
        for (Size j = 0; j < cols(); ++j)
        {
            fmt::print("-----------");
        }
        fmt::print("\n");

        for (Size i = 0; i < rows(); ++i)
        {
            fmt::print("{:>10} |", row_names[i]);
            for (Size j = 0; j < cols(); ++j)
            {
                if constexpr (std::is_same<T, bool>::value)
                {
                    fmt::print(" {:>10}", static_cast<int>(operator()(i, j)));
                }
                else if constexpr (std::is_same<T, float>::value || std::is_same<T, double>::value)
                {
                    fmt::print(" {:>10.2f}", operator()(i, j));
                }
                else
                {
                    fmt::print(" {:>10}", operator()(i, j));
                }
            }
            fmt::print("\n");
        }
        fmt::print("\n");
        fflush(stdout);
    }
};
