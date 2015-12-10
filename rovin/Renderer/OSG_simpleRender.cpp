#include "OSG_simpleRender.h"
#include <osgUtil/IncrementalCompileOperation>

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
			osg::ref_ptr< osg::MatrixTransform > linkPositionTransform;
			osg::ref_ptr< osg::MatrixTransform > scaleTransform;

			switch (geoPtr->getType())
			{
			case Model::GeometryInfo::GEOMETRY_TYPE::_BOX:
			{
				linkPositionTransform = new osg::MatrixTransform;
				linkPositionTransform->setMatrix(convertMatrix(geoPtr->getTransform()));
				Math::Vector3 dim = (std::static_pointer_cast<Model::Box>(geoPtr))->getDimension();
				linkPositionTransform->addChild(new osg::ShapeDrawable(new osg::Box(osg::Vec3(), (float)dim(1), (float)dim(0), (float)dim(2))));
				return linkPositionTransform;
			}

			case Model::GeometryInfo::GEOMETRY_TYPE::_SPHERE:
			{
				linkPositionTransform = new osg::MatrixTransform;
				linkPositionTransform->setMatrix(convertMatrix(geoPtr->getTransform()));
				linkPositionTransform->addChild(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), (float)(std::static_pointer_cast<Model::Sphere>(geoPtr))->getRadius())));
				return linkPositionTransform;
			}

			case Model::GeometryInfo::GEOMETRY_TYPE::_CAPSULE:
			{
				linkPositionTransform = new osg::MatrixTransform;
				linkPositionTransform->setMatrix(convertMatrix(geoPtr->getTransform()));
				Math::Vector2 dim = (std::static_pointer_cast<Model::Capsule>(geoPtr))->getDimension();
				linkPositionTransform->addChild(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3(), (float)dim(0), (float)dim(1))));
				return linkPositionTransform;
			}

			case Model::GeometryInfo::GEOMETRY_TYPE::_CYLINDER:
			{
				linkPositionTransform = new osg::MatrixTransform;
				linkPositionTransform->setMatrix(convertMatrix(geoPtr->getTransform()));
				Math::Vector2 dim = (std::static_pointer_cast<Model::Cylinder>(geoPtr))->getDimension();
				linkPositionTransform->addChild(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(), (float)dim(0), (float)dim(1))));
				return linkPositionTransform;
			}

			case Model::GeometryInfo::GEOMETRY_TYPE::_MESH:
			{
				osg::ref_ptr<osgDB::ReaderWriter::Options> _options = new osgDB::ReaderWriter::Options();
				_options->setObjectCacheHint(osgDB::ReaderWriter::Options::CACHE_ALL);
				auto meshPtr = std::static_pointer_cast<Model::Mesh>(geoPtr);

				linkPositionTransform = new osg::MatrixTransform;
				linkPositionTransform->setMatrix(convertMatrix(geoPtr->getTransform()));

				scaleTransform = new osg::MatrixTransform;
				scaleTransform->setMatrix(osg::Matrixd::scale(meshPtr->getDimension(), meshPtr->getDimension(), meshPtr->getDimension()));
				scaleTransform->getOrCreateStateSet()->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);

				osg::ref_ptr<osg::Node> STL_node = osgDB::readNodeFile(meshPtr->getUrl(), _options);

				STL_node->setCullingActive(true);
				STL_node->setDataVariance(osg::Object::DataVariance::STATIC);
				auto meshColor = meshPtr->getColor();

				OSG_NodeVisitor _nodeVisitor;
				_nodeVisitor.setColor(meshColor[0], meshColor[1], meshColor[2], meshColor[3]);
				STL_node->accept(_nodeVisitor);

				scaleTransform->addChild(STL_node);
				linkPositionTransform->addChild(scaleTransform);
				return linkPositionTransform;
			}

			default:
			{
				linkPositionTransform = new osg::MatrixTransform;
				// TODO
				return linkPositionTransform;
			}
			}
		}

		bool OSG_simpleRender::SimpleGUIHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
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

		OSG_simpleRender::OSG_simpleRender(const Model::Assembly& assem, const Model::State& state, int width, int height)
		{
			_cameraManipulator = new osgGA::TerrainManipulator();
			_rootNode = new osg::Group;

			osg::ref_ptr< osg::MatrixTransform > rootGroup = new osg::MatrixTransform;
			rootGroup->addChild(createGround());

			std::vector< Model::LinkPtr > linkPtr = assem.getLinkList();
			std::vector<Model::GeometryInfoPtr>& shapes = std::vector<Model::GeometryInfoPtr>();
			for (unsigned int i = 0; i < linkPtr.size(); i++)
			{
				const Model::State::LinkState &linkState = state.getLinkState(linkPtr[i]->getName());

				osg::ref_ptr< osg::MatrixTransform > transformNode = new osg::MatrixTransform;
				transformNode->setMatrix(convertMatrix(linkState._T));
				shapes = linkPtr[i]->getDrawingShapes();
				for (unsigned int j = 0; j < shapes.size(); j++)
					transformNode->addChild(convertGeo2Node(shapes[j]));
				_NodeStateList.push_back(NodeStatePair(transformNode, &linkState));

				rootGroup->addChild(transformNode);
			}
			//_optimzer.optimize(rootGroup);
			_rootNode->addChild(rootGroup);
			_optimzer.optimize(_rootNode);

			rootGroup->setMatrix(osg::Matrix(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, -0.7f, 1));
			_cameraManipulator->setDistance(5.0f);
			_cameraManipulator->setRotation(osg::Quat(1, osg::Vec3d(1, 0, 0)));

			osg::ref_ptr<osgUtil::IncrementalCompileOperation> ico = new osgUtil::IncrementalCompileOperation;
			ico->add(rootGroup);
			ico->release();
			_viewer.setIncrementalCompileOperation(ico);
			_viewer.setUpViewInWindow(40, 40, width, height);
			_viewer.setSceneData(_rootNode);
			_viewer.getCamera()->setClearColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
			_viewer.setCameraManipulator(_cameraManipulator.get(), true);
			_viewer.addEventHandler(new osgViewer::WindowSizeHandler);
			_viewer.addEventHandler(new SimpleGUIHandler());
		}

		osg::ref_ptr< osg::Node > OSG_simpleRender::createGround(const float& size)
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

							float length = std::sqrtf((float)((sy - numTiles)*(sy - numTiles) + (sx - numTiles)*(sx - numTiles)));
							float kapa = 5.0f / (numTiles*std::sqrtf(2) - numTiles);
							colors->push_back(osg::Vec4(color, 0.7f / (1 + std::expf(-kapa*(numTiles - length)))));
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
		void OSG_simpleRender::updateFrame()
		{
			for (unsigned int i = 0; i < _NodeStateList.size(); i++)
			{
				_NodeStateList[i].first->setMatrix(convertMatrix(_NodeStateList[i].second->_T));
			}
			_viewer.frame();
		}
	}
}