#pragma once

#include <osg/io_utils>
#include <osg/ShapeDrawable>
#include <osg/LineWidth>

namespace rovin
{
	namespace Renderer
	{
		class Primitives
		{
		public:
			Primitives(osg::Vec4 color = osg::Vec4(0.0, 0.0, 0.0, 1.0))
			{
				_geometry = new osg::Geometry();
				_points = new osg::Vec3Array;
				_color = new osg::Vec4Array;
				_color->push_back(color);
				_color->setBinding(osg::Array::BIND_OVERALL);
				_geometry->setVertexArray(_points);
				_geometry->setColorArray(_color);
			}

			void setColor(float r, float g, float b, float a = 1.0)
			{
				(*_color)[0][0] = r;
				(*_color)[0][1] = g;
				(*_color)[0][2] = b;
				(*_color)[0][3] = a;
			}

			const osg::ref_ptr<osg::Geometry>&	getGeometry() const
			{
				return _geometry;
			}

		protected:
			osg::ref_ptr<osg::Geometry>			_geometry;
			osg::ref_ptr<osg::Vec3Array>		_points;
			osg::ref_ptr<osg::Vec4Array>		_color;
			osg::ref_ptr<osg::DrawArrays>		_drawArrays;
		};


		class Line : public Primitives
		{
		public:
			Line(float lineWidth = 1, osg::Vec4 color = osg::Vec4(0.0, 0.0, 0.0, 1.0))
				: Primitives(color)
			{
				_lineWidth = new osg::LineWidth(lineWidth);
				_drawArrays = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP);
				_geometry->getOrCreateStateSet()->setAttributeAndModes(_lineWidth, osg::StateAttribute::ON);
				_geometry->addPrimitiveSet(_drawArrays);
				
			}

			void setLineWidth(float w)
			{
				_lineWidth->setWidth(w);
			}

			void push_back(const osg::Vec3&	point)
			{
				_points->push_back(point);
				_drawArrays->setFirst(0);
				_drawArrays->setCount(_points->size());
				_geometry->setVertexArray(_points);
			}

		protected:
			osg::ref_ptr<osg::LineWidth>		_lineWidth;
		};
	}
}