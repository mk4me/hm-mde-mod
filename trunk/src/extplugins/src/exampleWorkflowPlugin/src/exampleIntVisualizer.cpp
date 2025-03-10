#include "PCH.h"
#include "exampleIntVisualizer.h"
#include "Plugin.h"

ExampleIntVisualizer::ExampleIntVisualizer() :
activeSerie(nullptr)
{

}

plugin::IVisualizer * ExampleIntVisualizer::create() const
{
	return new ExampleIntVisualizer();
}

QWidget* ExampleIntVisualizer::createWidget()
{
	widget = new QWidget();
	QVBoxLayout * layout = new QVBoxLayout();
	widget->setLayout(layout);
	return widget;
}

QIcon* ExampleIntVisualizer::createIcon()
{
	return new QIcon();
}

QPixmap ExampleIntVisualizer::takeScreenshot() const
{
	return QPixmap::grabWidget(widget);
}

void ExampleIntVisualizer::update( double deltaTime )
{
	
}

plugin::IVisualizer::ISerie* ExampleIntVisualizer::createSerie(const ISerie* serie, const utils::TypeInfo & requestedType, const core::VariantConstPtr & data)
{
	throw std::logic_error("The method or operation is not implemented.");
}

plugin::IVisualizer::ISerie* ExampleIntVisualizer::createSerie(const utils::TypeInfo & requestedType, const core::VariantConstPtr & data)
{
	QListView* listView = new QListView();
	widget->layout()->addWidget(listView);
	IntSerie * serie = new IntSerie(listView);
	serie->setName("Int serie");
	serie->setData(requestedType, data);
	return serie;
}

plugin::IVisualizer::ISerie* ExampleIntVisualizer::createSerie( const ISerie* serie )
{
	throw std::logic_error("The method or operation is not implemented.");
}

void ExampleIntVisualizer::removeSerie( ISerie* serie )
{
	IntSerie * s = dynamic_cast<IntSerie*>(serie);
	widget->layout()->removeWidget(s->view);
	s->view = nullptr;
}

void ExampleIntVisualizer::setActiveSerie( ISerie * serie )
{
	activeSerie = serie;	
}

const plugin::IVisualizer::ISerie * ExampleIntVisualizer::getActiveSerie() const
{
	return activeSerie;
}

plugin::IVisualizer::ISerie * ExampleIntVisualizer::getActiveSerie()
{
	return activeSerie;
}

void ExampleIntVisualizer::getSupportedTypes( utils::TypeInfoList & supportedTypes ) const
{
	supportedTypes.push_back(typeid(Ints));
}

int ExampleIntVisualizer::getMaxDataSeries() const
{
	return -1;
}


ExampleIntVisualizer::IntSerie::IntSerie( QListView * view) :
	view(view)
{

}


void ExampleIntVisualizer::IntSerie::setupData(const core::VariantConstPtr & data)
{
    IntsConstPtr cints = data->get();
    // w tym przypadku skorzystalismy z klasy IntsModel, stad potrzeba kopiowania obiektu
    // zwykle nie jest ono konieczne, w koncu mozna wizualizowac obiekty const
    IntsPtr ints = IntsPtr(new Ints(*cints));
    IntsModel* model = new IntsModel(ints);
    view->setModel(model);
}

void ExampleIntVisualizer::IntSerie::update()
{

}

