/* test.h - tiny dependency-free test harness. */
#ifndef KESTREL_TEST_H
#define KESTREL_TEST_H

#include <math.h>
#include <stdio.h>

static int t_failures, t_checks;

#define CHECK(cond)                                                              \
    do {                                                                         \
        t_checks++;                                                              \
        if (!(cond)) {                                                           \
            t_failures++;                                                        \
            fprintf(stderr, "  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
        }                                                                        \
    } while (0)

#define CHECK_EQ(a, b)                                                                           \
    do {                                                                                         \
        t_checks++;                                                                              \
        long long _a = (long long)(a), _b = (long long)(b);                                      \
        if (_a != _b) {                                                                          \
            t_failures++;                                                                        \
            fprintf(stderr, "  FAIL %s:%d: %s == %s (%lld vs %lld)\n", __FILE__, __LINE__, #a, \
                    #b, _a, _b);                                                                 \
        }                                                                                        \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                \
    do {                                                                                     \
        t_checks++;                                                                          \
        double _a = (double)(a), _b = (double)(b);                                           \
        if (!(fabs(_a - _b) <= (tol))) {                                                     \
            t_failures++;                                                                    \
            fprintf(stderr, "  FAIL %s:%d: %s ~= %s (%g vs %g, tol %g)\n", __FILE__,       \
                    __LINE__, #a, #b, _a, _b, (double)(tol));                                \
        }                                                                                    \
    } while (0)

#define RUN(fn)                                       \
    do {                                              \
        int _before = t_failures;                     \
        fn();                                         \
        printf("%s %s\n", t_failures == _before ? "  ok  " : "  FAIL", #fn); \
    } while (0)

#endif
