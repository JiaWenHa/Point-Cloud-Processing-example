#ifndef MYVIZ_H
#define MYVIZ_H

#include <QWidget>
#include "rviz/add_display_dialog.h"
#include "rviz/displays_panel.h"

namespace rviz
{
class Display;
class Line;
class RenderPanel;
class DisplaysPanel;
class VisualizationManager;
class AddDisplayDialog;
}

// BEGIN_TUTORIAL
// Class "MyViz" implements the top level widget for this example.
class MyViz: public QWidget
{
Q_OBJECT
public:
  MyViz( QWidget* parent = 0 );
  virtual ~MyViz();

private Q_SLOTS:
  void setThickness( int thickness_percent );
  void setCellSize( int cell_size_percent );

private:
  rviz::VisualizationManager* manager_; //VisualizationManager类型指针 manager_ 用来创建 rviz 内已有的display
  rviz::RenderPanel* render_panel_; //render_panel_指针用来构建和布置渲染画板
  rviz::DisplaysPanel* displays_panel_;
  rviz::Display* grid_;
  rviz::Display* pc_;//Display类型指针 pc_ 即我们要显示的 PointCloud2 类型的display对象
};
// END_TUTORIAL
#endif // MYVIZ_H
