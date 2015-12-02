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

namespace rovin
{
	namespace Renderer
	{
		class SimpleOSG
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

			SimpleOSG(const Model::Assembly& assem, const Model::State& state, int width, int height);

			static const unsigned int numTiles = 25;
			static osg::ref_ptr< osg::Node > createGround(const float& size = (10.f));
			
		public:
			osgViewer::Viewer _viewer;
			osg::ref_ptr< osg::Group > _rootNode;
			osg::ref_ptr< osgGA::TerrainManipulator > _cameraManipulator;

			std::vector< NodeStatePair > _Link;
		};
	}
}