#include <QAbstractItemModel>
#include "HuboPlus.h"

#ifdef HAVE_HUBO_ACH
#include "HuboAchDriver.h"
#endif

class HuboDataModel: public QAbstractTableModel {
public:

  enum ColumnIndex {
    COL_NAME,
    COL_CMD_ANGLE,
    COL_CUR_ANGLE,
    COL_BLANK,
    NUM_COLS
  };


  HuboDataModel(const HuboPlus& h, QObject* parent=0);

  virtual ~HuboDataModel();

  virtual int rowCount(const QModelIndex& parent) const;

  virtual int columnCount(const QModelIndex& parent) const;

  virtual QVariant data(const QModelIndex& index, 
                        int role=Qt::DisplayRole) const;

  virtual bool setData(const QModelIndex& index, 
                       const QVariant& value,
                       int role=Qt::EditRole);

  virtual QVariant headerData(int section, 
                              Qt::Orientation orientation, 
                              int role=Qt::DisplayRole) const;

  virtual Qt::ItemFlags flags(const QModelIndex& index) const;

  void handleComms();

  void curUpdated(int minRow, int maxRow);
  
  void commandIK(fakerave::real footSep,
                 const fakerave::vec3& bodyPos,
                 const fakerave::quat& bodyRot);

  void updateIK();
                 

  const HuboPlus& hplus;

#ifdef HAVE_HUBO_ACH
  HuboAchDriver achDriver;
  fakerave::IndexArray hidx;
  hubo_ref_t href;
#endif

  std::vector<std::string> jnames;
  fakerave::IndexArray jidx;

  HuboPlus::KState cmdState;
  HuboPlus::KState curState;

  bool legIKEnabled;

  fakerave::real defaultFootSep;
  fakerave::vec3 defaultBodyPos;
  fakerave::quat defaultBodyRot;

  fakerave::vec3 ikBodyPos;
  fakerave::quat ikBodyRot;



};
