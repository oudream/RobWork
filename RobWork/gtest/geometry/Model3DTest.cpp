
#include "../TestEnvironment.hpp"

#include <rw/geometry/Model3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <gtest/gtest.h>
#include <string>

using namespace rw::geometry;
using namespace rw::math;

TEST (Model3D, constructor)
{
    Model3D m ("Vary Nice");
    EXPECT_EQ (m.getName (), "Vary Nice");

    EXPECT_EQ (m.isDynamic (), false);
    EXPECT_EQ (m.getMask (), 0);
    EXPECT_EQ (m.getFilePath (), "");
    EXPECT_EQ (m.getTransform (), Transform3D< double > ());
    EXPECT_EQ (m.getObjects ().size (), 0ul);
}