//
// Authror: Randy Fawcett on 06/2021.
//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#ifndef __OGRE_HELPER__
#define __OGRE_HELPER__

#include "raisim/OgreVis.hpp"
#include "Ogre.h"
#include "string"
#include "iostream"

void printAllBodyNames(raisim::OgreVis *vis){
    // This function prints the names of all of the visual bodies being rendered in the
    // Ogre environment

    Ogre::SceneManager *scmg = vis->getSceneManager(); 
    Ogre::SceneManager::MovableObjectIterator it = scmg->getMovableObjectIterator("Entity");
    while(it.hasMoreElements()){
        std::cout<<it.getNext()->getName()<<std::endl;
    }
}

void assignBodyColors(raisim::OgreVis *vis, std::string has_str, std::string excludes_str, std::string mat_str){
    // This function changes the material of certain bodies in the Ogre environment.
    // The function changes the bodies with names that contain has_str, but excludes
    // those that contain exclude_str.
    //
    // inputs:
    //      has_str: The string that the name of the body must include
    //      exclude_str: The string that the name of the body must exclude
    //      mat_str: String name of the material to be applied
    //
    
    Ogre::SceneManager *scmg = vis->getSceneManager(); 
    Ogre::SceneManager::MovableObjectIterator it = scmg->getMovableObjectIterator("Entity");
    while(it.hasMoreElements()){
        std::string name = it.getNext()->getName();
        int index, pos = 0, colpos = 0;
        while( (index = name.find(has_str,pos) ) != std::string::npos){pos = index+1; break;}
        if(pos != 0){
            while( (index = name.find(excludes_str,colpos) ) != std::string::npos){colpos = index+1; break;}
        }
        if(pos!=0 && colpos==0){
            Ogre::Entity *bod = scmg->getEntity(name);
            bod->setMaterialName(mat_str);
        }
    }
}

void assignBodyColors(raisim::OgreVis *vis, std::string body_name, std::string mat_str){
    // This function changes the material of a specific body (given by body_name)
    //
    // inputs:
    //      body_name: The name of the body to which the material should be applied
    //      mat_str: String name of the material to be applied
    //

    Ogre::SceneManager *scmg = vis->getSceneManager(); 
    if (!scmg->hasEntity(body_name)){
        std::cout << "The provided body_name (during the call to assignBodyColors)\ndoes not exist"<<std::endl;
        return;
    }
    Ogre::Entity *bod = scmg->getEntity(body_name);
    bod->setMaterialName(mat_str);
}

#endif