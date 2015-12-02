#include "SimpleOSG.h"

#include <cmath>

namespace rovin
{
	namespace Renderer
	{
		osg::Matrix convertMatrix(const Math::SE3 T)
		{
			Math::Matrix3 R = T.getRotation().matrix();
			Math::Vector3 p = T.getPosition();

			return osg::Matrix((float)R(0, 0), (float)R(1, 0), (float)R(2, 0), 0.0f, 
				(float)R(0, 1), (float)R(1, 1), (float)R(2, 1), 0.0f, 
				(float)R(0, 2), (float)R(1, 2), (float)R(2, 2), 0.0f, 
				(float)p(0), (float)p(1), (float)p(2), 1.0f);
		}

		osg::ref_ptr< osg::Node > convertGeo2Node(const Model::GeometryInfoPtr geoPtr)
		{
			osg::ref_ptr< osg::MatrixTransform > transformNode;

			switch (geoPtr->getType())
			{
			case Model::GeometryInfo::GEOMETRY_TYPE::_BOX:
			{
				transformNode = new osg::MatrixTransform;
				transformNode->setMatrix(convertMatrix(geoPtr->getTransform()));
				Math::Vector3 dim = (std::static_pointer_cast<Model::Box>(geoPtr))->getDimension();
				transformNode->addChild(new osg::ShapeDrawable(new osg::Box(osg::Vec3(), (float)dim(1), (float)dim(0), (float)dim(2))));
				return transformNode;
			}

			case Model::GeometryInfo::GEOMETRY_TYPE::_SPHERE:
			{
				transformNode = new osg::MatrixTransform;
				transformNode->setMatrix(convertMatrix(geoPtr->getTransform()));
				transformNode->addChild(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), (float)(std::static_pointer_cast<Model::Sphere>(geoPtr))->getRadius())));
				return transformNode;
			}

			case Model::GeometryInfo::GEOMETRY_TYPE::_CAPSULE:
			{
				transformNode = new osg::MatrixTransform;
				transformNode->setMatrix(convertMatrix(geoPtr->getTransform()));
				Math::Vector2 dim = (std::static_pointer_cast<Model::Capsule>(geoPtr))->getDimension();
				transformNode->addChild(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3(), (float)dim(0), (float)dim(1))));
				return transformNode;
			}

			case Model::GeometryInfo::GEOMETRY_TYPE::_CYLINDER:
			{
				transformNode = new osg::MatrixTransform;
				transformNode->setMatrix(convertMatrix(geoPtr->getTransform()));
				Math::Vector2 dim = (std::static_pointer_cast<Model::Cylinder>(geoPtr))->getDimension();
				transformNode->addChild(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(), (float)dim(0), (float)dim(1))));
				return transformNode;
			}

			case Model::GeometryInfo::GEOMETRY_TYPE::_MESH:
			{
				transformNode = new osg::MatrixTransform;
				// TODO
				return transformNode;
			}

			default:
			{
				transformNode = new osg::MatrixTransform;
				// TODO
				return transformNode;
			}
			}
		}

		bool SimpleOSG::SimpleGUIHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
		{
			switch (ea.getEventType())
			{
			case(osgGA::GUIEventAdapter::DRAG) :
			{
				if (_mouseRightButton)
				{
					return true;
				}
				return false;
			}
			case(osgGA::GUIEventAdapter::PUSH) :
			{
				if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
				{
					_mouseRightButton = true;
					return true;
				}
				return false;
			}
			case(osgGA::GUIEventAdapter::RELEASE) :
			{
				if (_mouseRightButton)
				{
					_mouseRightButton = false;
				}
				return true;
			}
			default:
				return false;
			}
		}

		SimpleOSG::SimpleOSG(const Model::Assembly& assem, const Model::State& state, int width, int height)
		{
			_cameraManipulator = new osgGA::TerrainManipulator();
			_rootNode = new osg::Group;

			_rootNode->addChild(createGround());

			osgUtil::Optimizer optimzer;
			optimzer.optimize(_rootNode);

			std::vector< Model::LinkPtr > linkPtr = assem.getLinkList();
			for (unsigned int i = 0; i < linkPtr.size(); i++)
			{
				const Model::State::LinkState &linkState = state.getLinkState(linkPtr[i]->getName());

				osg::ref_ptr< osg::MatrixTransform > transformNode = new osg::MatrixTransform;
				transformNode->setMatrix(convertMatrix(linkState._T));
				transformNode->addChild(convertGeo2Node(linkPtr[i]->getVisualGeometry()));
				_Link.push_back(NodeStatePair(transformNode, &linkState));

				_rootNode->addChild(transformNode);
			}

			_viewer.setUpViewInWindow(40, 40, width, height);
			_viewer.setSceneData(_rootNode);
			_viewer.getCamera()->setClearColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
			_viewer.setCameraManipulator(_cameraManipulator.get(), true);
			_viewer.addEventHandler(new osgViewer::WindowSizeHandler);
			_viewer.addEventHandler(new SimpleGUIHandler());
		}

		osg::ref_ptr< osg::Node > SimpleOSG::createGround(const float& size)
		{
			osg::ref_ptr< osg::Geode > geode = new osg::Geode;
			osg::ref_ptr< osg::Geometry > geom = new osg::Geometry;

			float width = 2 * size;
			float height = 2 * size;

			osg::Vec3 v0(-width*0.5f, -height*0.5f, 0.0f);
			osg::Vec3 dx(width / ((float)(numTiles * 2)), 0.0f, 0.0f);
			osg::Vec3 dy(0.0f, width / ((float)(numTiles * 2)), 0.0f);

			osg::Vec3Array* coords = new osg::Vec3Array;
			for (unsigned int iy = 0; iy <= numTiles * 2; iy++)
			{
				for (unsigned int ix = 0; ix <= numTiles * 2; ix++)
				{
					coords->push_back(v0 + dx*(float)ix + dy*(float)iy);
				}
			}
			geom->setVertexArray(coords);

			osg::ref_ptr< osg::Vec4Array > colors = new osg::Vec4Array;

			int numIndicesPerRow = numTiles * 2 + 1;
			for (unsigned int iy = 0; iy < numTiles; iy++)
			{
				for (unsigned int ix = 0; ix < numTiles; ix++)
				{
					osg::Vec3 color;
					color = ((iy + ix) % 2 == 0) ? osg::Vec3(1.0f, 1.0f, 1.0f) : osg::Vec3(0.2f, 0.2f, 0.2f);
					
					for (unsigned int sy = iy * 2; sy < (iy + 1) * 2; sy++)
					{
						for (unsigned int sx = ix * 2; sx < (ix + 1) * 2; sx++)
						{
							osg::ref_ptr< osg::DrawElementsUShort > primitives = new osg::DrawElementsUShort(GL_QUADS);
							primitives->push_back(sx + (sy + 1)*numIndicesPerRow);
							primitives->push_back(sx + sy*numIndicesPerRow);
							primitives->push_back((sx + 1) + sy*numIndicesPerRow);
							primitives->push_back((sx + 1) + (sy + 1)*numIndicesPerRow);
							geom->addPrimitiveSet(primitives.get());

							float length = std::sqrtf((float)((sy- numTiles)*(sy- numTiles) + (sx- numTiles)*(sx- numTiles)));
							float kapa = 5.0f / (numTiles*std::sqrtf(2) - numTiles);
							colors->push_back(osg::Vec4(color, 0.7f/(1 + std::expf(-kapa*(numTiles - length)))));
						}
					}
				}
			}

			geom->setColorArray(colors, osg::Array::BIND_PER_PRIMITIVE_SET);

			osg::ref_ptr< osg::Vec3Array > normals = new osg::Vec3Array;
			normals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
			geom->setNormalArray(normals, osg::Array::BIND_OVERALL);

			osg::StateSet* stateset = new osg::StateSet;
			stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
			stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
			stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
			geode->setStateSet(stateset);

			geode->addDrawable(geom);

			return geode;
		}
	}
}