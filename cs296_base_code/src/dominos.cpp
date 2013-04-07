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

#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs296
{
  dominos_t::dominos_t()
  {
    float32 heightOfWall = -2.0f;
    float32 distFromLeft = 5.0f;
//Ground
/*! \b Ground :-
 *      - Variables:
 *            - \c b2Body \a b1 : Creates a rigid body via b2World::CreateBody
 *            - \c b2EdgeShape \a shape : Creates a line segment.
 *            - \c b2BodyDef \a bd : Holds all the data of the rigid body.
 *
 *      - Creates the horizontal line at 0 y co-ordinate.
 */
    b2Body* b1;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
	
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
      

//Top horizontal shelf
/*! \b Top \b Horizontal \b shelf :-
 *  - Variables:
 *            - \c b2PolygonShape \a shape : Creates a convex polygon.
 *            - \c b2BodyDef \a bd : Holds all the data of the rigid body.
 *            - \c b2Body \a ground : Creates a rigid body via b2World::CreateBody
 *  
 *  - Creates the shelf on which the dominos are placed of size(12,0.5) at position(-31,30).
 */  
    {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);
	
      b2BodyDef bd;   
      bd.position.Set(-31.0f+distFromLeft, 30.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

//Dominos
/*! \b Dominos :-
 * - Variables:
 *            - \c b2PolygonShape \a shape : Creates a convex polygon.
 *            - \c b2FixtureDef \a fd : Used to create a fixture.
 *            - \c b2BodyDef \a bd : Holds all the data of the rigid body.
 *            - \c b2Body \a body : Creates a rigid body via b2World::CreateBody
 *  
 * - Creates ten dominos on the top shelf of size(0.2,1).
 */
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);
	
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
		
      for (int i = 0; i < 8; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f + 1.0f * i + distFromLeft, 31.25f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
    }
    
    //! Ball on the top shelf
    {
      b2Body* spherebody;
	
      b2CircleShape circle;
      circle.m_radius = 0.5;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
	
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-35.5f + 9.0f + distFromLeft, 31.25f);
      spherebody = m_world->CreateBody(&ballbd);
      spherebody->CreateFixture(&ballfd);
    }
    
    //! Vertical partition
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.25f, 16.0f-heightOfWall);
	
      b2BodyDef bd;   
      bd.position.Set(-23.0f + distFromLeft, 30.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    
    //! Tilted platform
    {
      b2Vec2 a;
      a.Set(0.0f,0.0f);
      b2PolygonShape shape;
      shape.SetAsBox(8.0f, 0.25f,a,0.125f);
      b2BodyDef bd;   
      bd.position.Set(-31.0f + distFromLeft, 15.0f+heightOfWall);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    
    //! Vertical chain of rotating walls 
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.25f, 1.25f);
      
      b2PolygonShape shape2;
      shape2.SetAsBox(1.25f, 0.25f);
	
      for (int i=0;i<3;i++){
        b2BodyDef bd;   
        bd.position.Set(-33.0f + distFromLeft, 17.0f + heightOfWall + 3.8*i);
        bd.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd);
        b2FixtureDef *fd = new b2FixtureDef;
        fd->density = 1.f;
        fd->shape = new b2PolygonShape;
        fd->shape = &shape;
        body->CreateFixture(fd);
        
        
        b2BodyDef bd2;
        bd2.position.Set(-33.0f + distFromLeft, 17.0f + heightOfWall + 3.8*i);
        b2Body* body2 = m_world->CreateBody(&bd2);

        b2RevoluteJointDef jointDef;
        jointDef.bodyA = body;
        jointDef.bodyB = body2;
        jointDef.localAnchorA.Set(0,0);
        jointDef.localAnchorB.Set(0,0);
        jointDef.collideConnected = false;
        m_world->CreateJoint(&jointDef);
      }
      for (int i=0;i<3;i++){
        b2BodyDef bd;   
        bd.position.Set(-32.0f + distFromLeft, 19.0f + heightOfWall + 3.8*i);
        bd.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd);
        b2FixtureDef *fd = new b2FixtureDef;
        fd->density = 1.f;
        fd->shape = new b2PolygonShape;
        fd->shape = &shape;
        body->CreateFixture(fd);
        
        b2BodyDef bd2;
        bd2.position.Set(-32.0f + distFromLeft, 19.0f + heightOfWall + 3.8*i);
        b2Body* body2 = m_world->CreateBody(&bd2);

        b2RevoluteJointDef jointDef;
        jointDef.bodyA = body;
        jointDef.bodyB = body2;
        jointDef.localAnchorA.Set(0,0);
        jointDef.localAnchorB.Set(0,0);
        jointDef.collideConnected = false;
        jointDef.maxMotorTorque = 1000.0f;
        m_world->CreateJoint(&jointDef);

      }
      //ground->CreateFixture(&shape, 0.0f);
    }
    
    //! Platform for the slider
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.2f, 0.25f);
	
      b2BodyDef bd;   
      bd.position.Set(-33.2f + distFromLeft, 26.6f + heightOfWall);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    
    //! Ball on the slider
    {
      b2Body* spherebody;
	
      b2CircleShape circle;
      circle.m_radius = 0.5;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
	
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-33.2f + distFromLeft, 27.6f + heightOfWall);
      spherebody = m_world->CreateBody(&ballbd);
      spherebody->CreateFixture(&ballfd);
    }
    
    //! Tilted platform for slider
    {
      b2Vec2 a;
      a.Set(0.0f,0.0f);
      b2PolygonShape shape;
      shape.SetAsBox(4.3f, 0.25f,a,0.125f);
      b2BodyDef bd;   
      bd.position.Set(-37.8f + distFromLeft, 26.2f + heightOfWall);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    
    //! Unstable hinged block
    {
      b2Vec2 vertices[3];
      vertices[0].Set(0.0f, 0.0f);
      vertices[1].Set(0.5f, 10.0f);
      vertices[2].Set(-0.5f, 10.0f);
      int32 count = 3;
      b2PolygonShape polygon;
      polygon.Set(vertices, count);
      b2BodyDef bd;   
      bd.position.Set(-42.8f + distFromLeft, 12.0f + heightOfWall);
      bd.type = b2_dynamicBody;
      b2Body* ground = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &polygon;
      ground->CreateFixture(fd);
      ground->CreateFixture(&polygon, 0.0f);
      
      b2PolygonShape shape2;
      shape2.SetAsBox(0.25f, 4.25f);
      b2BodyDef bd2;
      bd2.position.Set(-42.8f + distFromLeft, 17.0f + heightOfWall);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = ground;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);

    }
    
    //! Platform for the block which flies
    {
      b2PolygonShape shape;
      shape.SetAsBox(3.5f, 3.5f);
	
      b2BodyDef bd;   
      bd.position.Set(-54.0f + distFromLeft, 13.6f + heightOfWall);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    
    //! The block which flies
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.5f, 0.2f);
      
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 0.5f;
      fd.friction = 0.1f;
      fd.restitution = 0.0f;
            
      b2BodyDef bd;   
      bd.type = b2_dynamicBody;
      bd.position.Set(-51.0f + distFromLeft, 18.8f + heightOfWall);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&fd);
    }
    
    
    
    
    /*
    b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
    b2BodyDef bd;
      bd.position.Set(14.0f, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(14.0f, 16.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;

      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
*/

    
    
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      
//Another horizontal shelf
/*! \b Another \b Horizontal \b shelf :-
 * - Variables:
 *            - \c b2PolygonShape \a shape : Creates a convex polygon.
 *            - \c b2BodyDef \a bd : Holds all the data of the rigid body.
 *            - \c b2Body \a ground : Creates a rigid body via b2World::CreateBody
 *  
 * - Creates a horizontal below the top horizontal shelf for the train of spheres of size(14,0.5) and at position(-20,20).
 */
 
 /*
    {
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);
	
      b2BodyDef bd;
      bd.position.Set(1.0f, 6.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

*/
//The pendulum that knocks the dominos off
/*! \b The \b pendulum \b that \b knocks \b the \b dominos \b off :-
 * - Variables:
 *            - \c b2Body \a b2, \a b4 : Creates a rigid body via b2World::CreateBody
 *            - \c b2PolygonShape \a shape : Creates a convex polygon.
 *            - \c b2BodyDef \a bd : Holds all the data of the rigid body.
 *            - \c b2RevoluteJointDef \a jd : Defines Revolute joint. This requires an anchor point where the bodies are joined and the initial relative angle for joint limits.
 *            - \c b2Vec2 \a anchor : A 2D vector.
 *  
 * - Creates a pendulum at the start of the top shelf.
 *            - It makes two bodies, b2 is static, while b4 is dynamic.
 *            - b4 rotates about the joint jd.
 */
   
   
    {
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 1.5f);
	  
	b2BodyDef bd;
	bd.position.Set(-36.5f + distFromLeft, 28.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }
	
      b2Body* b4;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.25f);
	  
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(-40.0f + distFromLeft, 33.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 2.0f);
      }
	
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-37.0f + distFromLeft, 40.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }
      
//The train of small spheres
/*! \b The \b train \b of \b small \b spheres :-
 * - Variables:
 *            - \c b2Body \a spherebody : Creates a rigid body via b2World::CreateBody
 *            - \c b2CircleShape \a circle : Creates a line segment.
 *            - \c b2FixtureDef \a ballfd : Used to create a fixture.
 *            - \c b2BodyDef \a ballbd : Holds all the data of the rigid body.
 *  
 * - Creates ten spheres on the second shelf of radius(1.0).
 */
  /*
  
    {
      b2Body* spherebody;
	
      b2CircleShape circle;
      circle.m_radius = 0.5;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
	
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(-22.2f + i*1.0, 26.6f);
	  spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);
	}
    }

//The pulley system
/*! \b The \b pulley \b system :-
 * - Variables:
 *            - \c b2BodyDef \a bd : Holds all the data of the rigid body.
 *  
 * - Elements:
 *            - \a The open box:
 *                        - \c b2FixtureDef \a fd1, \a fd2, \a fd3 : Used to create a fixture.
 *                        - \c b2PolygonShape \a bs1, \a bs2, \a bs3 : Creates a convex polygon.
 *                        - \c b2Body \a box1 : Creates a rigid body via b2World::CreateBody
 *            - \a The bar:
 *                        - \c b2Body \a box2 : Creates a rigid body via b2World::CreateBody.
 *            - \a The pulley joint:
 *                        - \c b2PulleyJointDef \a myjoint : Defines a pulley joint. This requires two ground anchors, two dynamic body anchor points, and a pulley ratio. 
 *                        - \c b2Vec2 \a worldAnchorOnBody1, \a worldAnchorOnBody2, \a worldAnchorGround1, \a worldAnchorGround2 : A 2D vector.
 * - Creates a pulley system constituting of
 *            - an open box (\c box1) made by bs1,bs2,bs3; 
 *            - a bar (\c box2); and
 *            - the pulley joints using the two b2Body, four anchor points, and ratio. 
 */
  
  /*
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-10,15);
      bd->fixedRotation = true;
      
      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //The bar
      bd->position.Set(10,15);	
      fd1->density = 34.0;	  
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(10, 15); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-10, 20); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(10, 20); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }

//The revolving horizontal platform
/*! \b The \b revolving \b horizontal \b platform :-
 * - Variables:
 *            - \c b2PolygonShape \a shape : Creates a convex polygon.
 *            - \c b2BodyDef \a bd, \a bd2 : Holds all the data of the rigid body.
 *            - \c b2Body \a body, \a body2 : Creates a rigid body via b2World::CreateBody.
 *            - \c b2FixtureDef \a fd : Used to create a fixture.
 *            - \c b2RevoluteJointDef \a jd : Defines Revolute joint. This requires an anchor point where the bodies are joined and the initial relative angle for joint limits.
 *  
 * - Creates a horizontal platform which rotates around the center using
 *            - two bodies of size(4.4,0.4) and at positions (14,14) and (14,16).
 *            - first body is horizontal, other one is invisible; it is used only for anchoring.
 */
  
  /*
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
	
      b2BodyDef bd;
      bd.position.Set(14.0f, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(14.0f, 16.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }

//The heavy sphere on the platform
/*! \b The \b heavy \b sphere \b on \b the \b platform :-
 * - Variables:
 *            - \c b2Body \a sbody : Creates a rigid body via b2World::CreateBody.
 *            - \c b2CircleShape \a circle : Creates a line segment.
 *            - \c b2FixtureDef \a ballfd : Used to create a fixture.
 *            - \c b2BodyDef \a ballbd : Holds all the data of the rigid body.
 *  
 * - Creates a heavy sphere of radius(2.0) at the center of the rotating platform.
 */
  
  /*
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(14.0f, 18.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }


//The see-saw system at the bottom
/*! \b The \b see-saw \b system \b at \b the \b bottom :-
 * - Elements:
 *            - \a The triangle wedge:
 *                        - \c b2Body \a sbody : Creates a rigid body via b2World::CreateBody
 *                        - \c b2PolygonShape \a poly : Creates a convex polygon.
 *                        - \c b2FixtureDef \a wedgefd : Used to create a fixture.
 *                        - \c b2BodyDef \a wedgebd : Holds all the data of the rigid body.
 *            - \a The plank on top of the wedge:
 *                        - \c b2PolygonShape \a shape : Creates a convex polygon.
 *                        - \c b2BodyDef \a bd2 : Holds all the data of the rigid body.
 *                        - \c b2Body \a body : Creates a rigid body via b2World::CreateBody.
 *                        - \c b2FixtureDef \a fd2 : Used to create a fixture.
 *                        - \c b2RevoluteJointDef \a jd : Defines Revolute joint. This requires an anchor point where the bodies are joined and the initial relative angle for joint limits.
 *                        - \c b2Vec2 \a anchor : A 2D vector.
 *            - \a The light box on the right side of the see-saw:
 *                        - \c b2PolygonShape \a shape2 : Creates a convex polygon.
 *                        - \c b2BodyDef \a bd3 : Holds all the data of the rigid body.
 *                        - \c b2Body \a body3 : Creates a rigid body via b2World::CreateBody.
 *                        - \c b2FixtureDef \a fd3 : Used to create a fixture.
 *
 * - Creates a see-saw constituting of
 *            - an triangle wedge (\c sbody); 
 *            - a plank (\c body); and
 *            - a light box (\c body3) to be placed on the see-saw. 
 */
   
   /*
    {
      //The triangle wedge
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,1.5);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(30.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);

      //The plank on top of the wedge
      b2PolygonShape shape;
      shape.SetAsBox(15.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(30.0f, 1.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      body->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(30.0f, 1.5f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      //The light box on the right side of the see-saw
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 2.0f);
      b2BodyDef bd3;
      bd3.position.Set(40.0f, 2.0f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.01f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
    }
    */
  }
  
  //! Initializes an object of sim_t
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
