#pragma once

#include <rovin/Math/Common.h>
#include <rovin/Math/LieGroup.h>
#include <rovin/Model/Assembly.h>

#include <osg/io_utils>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>
#include <osgUtil/Optimizer>
#include <osgGA/TerrainManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgDB/ReadFile>
#include <osg/LineWidth>
#include <osg/Material>

#include "OSG_NodeVisitor.h"
#include "OSG_Primitives.h"
#include "OSG_ExtFile.h"

namespace rovin
{
	namespace Renderer
	{
		class OSG_simpleRender
		{
			class SimpleGUIHandler : public osgGA::GUIEventHandler
			{
			public:
				SimpleGUIHandler() : _mouseRightButton(false) {}
				~SimpleGUIHandler() {}

				bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

			private:
				bool _mouseRightButton;
			};

		public:
			typedef std::pair< osg::ref_ptr< osg::MatrixTransform >, const Model::State::LinkState* > NodeStatePair;
			OSG_simpleRender(const Model::Assembly& assem, const Model::State& state, int width, int height);
			
			osg::ref_ptr< osg::Group >&	getRoot() { return _rootNode; }
			osgViewer::Viewer&	getViewer() { return _viewer; }
			void updateFrame();


			void addGeometry(const Primitives& geom) { _geometryNode->addDrawable(geom.getGeometry()); }
			void removeGeometry(const Primitives& geom) { _geometryNode->removeDrawable(geom.getGeometry()); }

			void add(const ExtFile& file) { _rootNode->addChild(file.getRoot()); }
			void remove(const ExtFile& file) { _rootNode->removeChild(file.getRoot()); }

		protected:
			static const unsigned int numTiles = 25;
			static osg::ref_ptr< osg::Node > createGround(const float& size = (1.0f));

			osg::ref_ptr< osg::Group >					_rootNode;
			osgViewer::Viewer							_viewer;
			osg::ref_ptr< osgGA::TerrainManipulator >	_cameraManipulator;
			osgUtil::Optimizer							_optimzer;
			osg::ref_ptr<osg::Geode>					_geometryNode;
			std::vector< NodeStatePair >				_NodeStateList;
		};
	}
}