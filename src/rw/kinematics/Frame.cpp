/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "Frame.hpp"

#include "State.hpp"
#include "TreeState.hpp"

#include <rw/common/macros.hpp>
#include <rw/common/Property.hpp>

#include <rw/kinematics/FKRange.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;

Frame::Frame(int dof, const std::string& name) :
    StateData(dof,name),
    _parent(NULL)
{
}

// Parents.

Frame* Frame::getParent(const State& state)
{
    Frame* f1 = getParent();
    if (f1)
        return f1;
    else
        return getDafParent(state);
}

const Frame* Frame::getParent(const State& state) const
{
    //std::cout << " getParent ";
    const Frame* f1 = getParent();
    if (f1)
        return f1;
    else
        return getDafParent(state);
}

const Frame* Frame::getDafParent(const State& state) const
{
    //std::cout << " getDAFParent ";
    return state.getTreeState().getParent(this);
}

Frame* Frame::getDafParent(const State& state)
{
    return state.getTreeState().getParent(this);
}

// Children.

Frame::const_iterator_pair Frame::getChildren(const State& state) const
{
    const std::vector<Frame*> *list = state.getTreeState().getChildren(this);
    if(list != NULL)
        return makeConstIteratorPair(_children, *list);
    return makeConstIteratorPair(_children);
}

Frame::iterator_pair Frame::getChildren(const State& state)
{
    const std::vector<Frame*> *list = state.getTreeState().getChildren(this);
    if(list != NULL)
        return makeIteratorPair(_children, *list);
    return makeIteratorPair(_children);
}

Frame::const_iterator_pair Frame::getDafChildren(const State& state) const
{
    const std::vector<Frame*> *list = state.getTreeState().getChildren(this);
    if(list!=NULL)
        return makeConstIteratorPair(*list);
    return makeConstIteratorPair(std::vector<Frame*>());// empty iterator
}

Frame::iterator_pair Frame::getDafChildren(const State& state)
{
    const std::vector<Frame*> *list = state.getTreeState().getChildren(this);
    if(list!=NULL)
        return makeIteratorPair(*list);
    return makeIteratorPair(std::vector<Frame*>());// empty iterator
}

// Frame attachments.

void Frame::attachTo(Frame* parent, State& state)
{
    state.getTreeState().attachFrame(this, parent);
}

std::ostream& rw::kinematics::operator<<(std::ostream& out, const Frame& frame)
{
    return out << "Frame[" << frame.getName() << "]";
}
