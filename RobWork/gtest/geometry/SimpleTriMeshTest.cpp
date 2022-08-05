/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include <RobWorkConfig.hpp>
#include <rw/geometry/Box.hpp>
#include <rw/geometry/GeometryUtil.hpp>
#include <rw/geometry/SimpleTriMesh.hpp>

#include <gtest/gtest.h>

using namespace rw::geometry;
using namespace rw::math;

TEST (SimpleTriMesh, ConstructionAndOverloads)
{
    Box box (1, 2, 3);

    SimpleTriMesh s1 (box);
    SimpleTriMesh s2 (box.createMesh (100));

    EXPECT_EQ (s1.vertices (), 8u);
    EXPECT_EQ (s1.triangles (), 12);
    EXPECT_EQ (s1.triangles (), s1.size ());

    EXPECT_EQ (s2.vertices (), 8u);
    EXPECT_EQ (s2.triangles (), 12);

    EXPECT_NEAR (GeometryUtil::estimateVolume (s1), 1 * 2 * 3.0, 10e-6);
    s2.scale (2);
    EXPECT_NEAR (GeometryUtil::estimateVolume (s2), (1 * 2 * 3.0) * pow (2, 3), 10e-6);

    SimpleTriMesh s3 = s1.clone ();
    for (size_t i = 0; i < s1.size (); i++) {
        Triangle< double > t1 = s1.getTriangle (i);
        Triangle< double > t2 = s2.getTriangle (i);
        Triangle< double > t3 = s3.getTriangle (i);

        EXPECT_EQ (t1[0], t3[0]);
        EXPECT_EQ (t1[1], t3[1]);
        EXPECT_EQ (t1[2], t3[2]);

        EXPECT_NE (t1[0], t2[0]);
        EXPECT_NE (t1[1], t2[1]);
        EXPECT_NE (t1[2], t2[2]);
    }
}

TEST (SimpleTriMesh, UNION)
{
    Box b1 (1, 2, 1);
    SimpleTriMesh s1 (b1);
    SimpleTriMesh s2 (b1);

    s2 *= Transform3D<> (Vector3D< double > (1, 0, 0));
    SimpleTriMesh s3 = s1 + s2;

    EXPECT_NEAR (GeometryUtil::estimateVolume (s3), (1 * 2 * 1) * 2, 10e-6);
}

TEST (SimpleTriMesh, DIFFERENCE)
{
    Box b1 (1, 2, 1);
    Box b2 (1, 1, 1);
    SimpleTriMesh s1 (b1);
    SimpleTriMesh s2 (b2);

    SimpleTriMesh s3 = s1 - s2;

    EXPECT_NEAR (GeometryUtil::estimateVolume (s3), 1, 10e-6);
}

TEST (SimpleTriMesh, INTERSECTION)
{
    Box b1 (1, 2, 1);
    Box b2 (2, 1, 1);
    SimpleTriMesh s1 (b1);
    SimpleTriMesh s2 (b2);

    SimpleTriMesh s3 = s1 & s2;

    EXPECT_NEAR (GeometryUtil::estimateVolume (s3), 1, 10e-6);
}

TEST (SimpleTriMesh, SYMETRICDIFFERENCE)
{
    Box b1 (1, 2, 1);
    Box b2 (1, 2, 1);
    SimpleTriMesh s1 (b1);
    SimpleTriMesh s2 (b2);
    s2 *= Transform3D<> (Vector3D< double > (0, 1, 0));

    SimpleTriMesh s3 = s1 ^ s2;

    EXPECT_NEAR (GeometryUtil::estimateVolume (s3), 2, 10e-6);
}

TEST (SimpleTriMesh, CombineAndSeperate)
{
    Box b1 (0.1, 0.1, 0.1);
    Box b2 (0.1, 0.1, 0.1);
    SimpleTriMesh s1 (b1);
    SimpleTriMesh s2 (b2);
    s2 *= Transform3D<> (Vector3D< double > (0, 0.2, 0));

    SimpleTriMesh s3 = s1.combine (s2);

    EXPECT_EQ (s3.vertices (), s1.vertices () + s2.vertices ());
    EXPECT_EQ (s3.triangles (), s1.triangles () + s2.triangles ());

    std::vector< SimpleTriMesh > sep = s3.separateMeshes ();

    EXPECT_EQ (sep.size (), 2u);

    EXPECT_EQ (s1.vertices (), sep[0].vertices ());
    EXPECT_EQ (s1.triangles (), sep[0].triangles ());
    EXPECT_EQ (s2.vertices (), sep[1].vertices ());
    EXPECT_EQ (s2.triangles (), sep[1].triangles ());
}