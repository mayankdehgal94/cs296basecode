/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */


#ifndef _CS296BASE_HPP_
#define _CS296BASE_HPP_

#include "render.hpp"
#include <Box2D/Box2D.h>
#include <cstdlib>

#define	RAND_LIMIT 32767

namespace cs296
{

  //! What is the difference between a class and a struct in C++?
  //! A class can have data members and their associated functions whereas a struct can have only data members
  class base_sim_t;
  
//! Simulation settings. Some can be controlled in the GUI.

  struct settings_t;
  
  //! Why do we use a typedef
  //! We use typedef to assign alternative names to existing types
  typedef base_sim_t* sim_create_fcn(); 

  //! Simulation settings. Some can be controlled in the GUI.
  //! This struct declares the member variables used in the graphical interface, the constructor initializes the variables as soon as an object of struct is created
  
  /*! This struct is basically to set the environmental conditions and simulate the objects created.

 *  - \b Variables:
 *      - b2Vec2 view_center : stores the position of the center of view of the GUI as 2D component vector. \n\n
 *      - float32 hz : stores the simulation steps per frame. \n\n
 *      - int32 velocity_iterations : Stores number of velocity iterations which is number of pass over velocity constraints in a single time step. \n\n
 *      - int32 position_iterations : Stores number of position iterations which is number of pass over position constraints in a single time step. \n\n
 *      - int32 draw_shapes : Used to draw shapes in the GUI \n\n
 *      - int32 draw_joints : Used to draw joints in the GUI \n\n
 *      - int32 draw_AABBs : Used to draw Axis aligned bounding boxes in the GUI \n\n
 *      - int32 draw_pairs : Used to draw pairs in the GUI \n\n
 *      - int32 draw_contact_points : Used to draw contact points in the GUI \n\n
 *      - int32 draw_contact_normals : Used to draw normal forces at contact points in the GUI \n\n
 *      - int32 draw_contact_forces : Used to draw forces at contact points in the GUI \n\n
 *      - int32 draw_friction_forces : Used to draw friction forces in the GUI \n\n
 *      - int32 draw_COMs : Used to draw center of mass of objects in the GUI \n\n
 *      - int32 draw_stats : Used to write statistics in the GUI \n\n
 *      - int32 draw_profile : Used to write the current profile(velocity, position, etc.) in the GUI \n\n
 *      - int32 enable_warm_starting : Used to enable warm starting in the GUI \n\n
 *      - int32 enable_continuous : Used to enable continous evaluation of variable of velocity,position,etc \n\n
 *      - int32 enable_sub_stepping : Used to enable sub stepping (steps can be taken within a single time step) \n\n
 *      - int32 pause : Used to pause \n\n
 *      - int32 single_step : Used to toggle step-by-step viewing. \n\n
 * 
 */

  struct settings_t
  {
    //! Notice the initialization of the class members in the constructor
    //! How is this happening?
    //!
    //! The constructor initializes the member variables, if they are initialized with 1, it means that the corresponding button would be checked else unchecked  
    //!
    //! declaring the required variables
    settings_t() :
      view_center(0.0f, 20.0f),
      hz(60.0f),
      velocity_iterations(8),
      position_iterations(3),
      draw_shapes(1),
      draw_joints(1),
      draw_AABBs(0),
      draw_pairs(0),
      draw_contact_points(0),
      draw_contact_normals(0),
      draw_contact_forces(0),
      draw_friction_forces(0),
      draw_COMs(0),
      draw_stats(0),
      draw_profile(0),
      enable_warm_starting(1),
      enable_continuous(1),
      enable_sub_stepping(0),
      pause(0),
      single_step(0)
    {}
    
    b2Vec2 view_center;
    float32 hz;
    int32 velocity_iterations;
    int32 position_iterations;
    int32 draw_shapes;
    int32 draw_joints;
    int32 draw_AABBs;
    int32 draw_pairs;
    int32 draw_contact_points;
    int32 draw_contact_normals;
    int32 draw_contact_forces;
    int32 draw_friction_forces;
    int32 draw_COMs;
    int32 draw_stats;
    int32 draw_profile;
    int32 enable_warm_starting;
    int32 enable_continuous;
    int32 enable_sub_stepping;
    int32 pause;
    int32 single_step;
  };
  
  //! This struct has spme data members and a consructor which takes values and initializes the members
  
  struct sim_t
  {
    const char *name;
    sim_create_fcn *create_fcn;

    sim_t(const char *_name, sim_create_fcn *_create_fcn): 
      name(_name), create_fcn(_create_fcn) {;}
  };
  
  extern sim_t *sim;
  
  //! declares fixture variables and 2d column vector variables 
  
  const int32 k_max_contact_points = 2048;  
  
/*! This struct defines a single contact point.
 *  - \b Variables :
 *      - b2Fixture* fixtureA,fixtureB : fixtures in the contact. \n \n
 *      - b2Vec2 normal : normal at contact point. \n \n
 *      - b2Vec2 position : position of the contact point. \n \n
 *      - b2PointState state : stores the state of points in contact.
 */ 
  struct contact_point_t
  {
    b2Fixture* fixtureA;
    b2Fixture* fixtureB;
    b2Vec2 normal;
    b2Vec2 position;
    b2PointState state;
  };
  
  //! This class is used to simulate contact information. It inherits b2ContactListener and is inherited by dominos_t class
  class base_sim_t : public b2ContactListener
  {
  public:
    
    base_sim_t();

    //! Virtual destructors - amazing objects. Why are these necessary?
    //!
    //! Virtual destructors are useful when we are inhereting a class
    //! The order of execution of the destructor would be
    //! 	- destructor of inheriting class
    //! 	- destructor of base class
    
    virtual ~base_sim_t();
    
    //! declaring set_text_line function
    //! @param [line] an integer variable passed as argument
    //! setting the value of the m-text_line variable equal to line
    
    void set_text_line(int32 line) { m_text_line = line; }
    
    //! declaring draw_title function
    //! @param [x] x coordinate of the position where the string is to be written
    //! @param [y] y coordinate of the position where the string is to be written
    //! @param [string] the string to be written 
    
    void draw_title(int x, int y, const char *string);
    
    //! declaring function step
    //! @param [settings] it is a pointer to settings_t struct which is passed as an argument
    //! 
    //! see settings_t
    virtual void step(settings_t* settings);
     
    //! declaring keyboard method
    //! @param [key] input key
    //! This method is to be called when any key is pressed.
    virtual void keyboard(unsigned char key) { B2_NOT_USED(key); }
    virtual void keyboard_up(unsigned char key) { B2_NOT_USED(key); }

    void shift_mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }
    virtual void mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }
    virtual void mouse_up(const b2Vec2& p) { B2_NOT_USED(p); }
    void mouse_move(const b2Vec2& p) { B2_NOT_USED(p); }

    
    //! Let derived tests know that a joint was destroyed.
    virtual void joint_destroyed(b2Joint* joint) { B2_NOT_USED(joint); }
    
    //! Callbacks for derived classes.
    virtual void begin_contact(b2Contact* contact) { B2_NOT_USED(contact); }
    
    //! Callbacks for derived classes.
    virtual void end_contact(b2Contact* contact) { B2_NOT_USED(contact); }
    
    
    //! declaring this as virtual function because this class is being inherited by dominos_t class
    //! @param [contact] a pointer to b2Contact object
    //! @param [oldManifold] a pointer to b2Manifold object
    //!
    //! see dominos_t
    virtual void pre_solve(b2Contact* contact, const b2Manifold* oldManifold);
    
    //! declaring this as virtual function because this class is being inherited by dominos_t class
    //! @param [contact] a pointer to b2Contact object
    //! @param [impulse] a pointer to b2ContactImpulse object
    //!
    //! see dominos_t
    virtual void post_solve(const b2Contact* contact, const b2ContactImpulse* impulse)
    {
      B2_NOT_USED(contact);
      B2_NOT_USED(impulse);
    }
  //!
  //!How are protected members different from private memebers of a class in C++ ?
  //!
  //!Private members are only accessible within the class defining them.
  //!
  //!Protected members are accessible in the class that defines them and in classes that inherit from that class.
  //!
  //
  
  protected:
    //!\n\n
    //! What are Friend classes?
    //!
    //! A Friend class is a class which can access the private and public members of the class which declared it as it's friend class
    friend class contact_listener_t;
    
    //! declaring a protected b2Body variable(i.e. a rigid body)
    b2Body* m_ground_body;
    
    //! declaring a b2AABB variable which would be used for drawing boundaries around obects
    b2AABB m_world_AABB;
    
    // !declaring an object of the struct contact_point_t 
    // !see contact_point_t
    contact_point_t m_points[k_max_contact_points];
    
    //! declaring an integer variable  
    int32 m_point_count;

    //! declaring an object of the class debug_draw_t
    //!
    //! see debug_draw_t
    debug_draw_t m_debug_draw;
    
    //! declaring an integer variable m_text_line
    int32 m_text_line;
    
    // creating an object of the b2world class which manages all entities
    b2World* m_world;

    int32 m_step_count;
    
    b2Profile m_max_profile;
    b2Profile m_total_profile;
  };
}

#endif
