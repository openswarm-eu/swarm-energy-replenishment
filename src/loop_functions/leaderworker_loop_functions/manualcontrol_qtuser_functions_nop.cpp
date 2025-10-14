#include "manualcontrol_qtuser_functions_nop.h"
#include <QKeyEvent>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>
#include <controllers/worker/worker.h>
#include <controllers/charger/charger.h>

/****************************************/
/****************************************/

static const Real DIRECTION_VECTOR_FACTOR = 10.;

/****************************************/
/****************************************/

CVector2 reduceLength(CVector2 vec, Real reduction) {
   Real length = std::sqrt(vec.GetX() * vec.GetX() + vec.GetY() * vec.GetY());
   if (length == 0.0){
      LOGERR << "Division by zero in reduceLength" << std::endl;
      return CVector2();  // Avoid division by zero
   }

   Real newLength = std::max(0.0, length - reduction);  // Ensure non-negative length
   Real scale = newLength / length;

   return CVector2(vec.GetX()*scale, vec.GetY()*scale);
}

/****************************************/
/****************************************/

CVector2 arrowTip(CVector2 point, Real distance, CRadians angle_rad) {
   Real dx = distance * Cos(angle_rad);
   Real dy = distance * Sin(angle_rad);
   return CVector2(point.GetX() + dx, point.GetY() + dy);
}

/****************************************/
/****************************************/

CManualControlQTUserFunctionsNop::CManualControlQTUserFunctionsNop() :
   m_pcController(NULL),
   m_strSendCommand("") {
   /* No key is pressed initially */
   m_punPressedKeys[DIRECTION_FORWARD]  = 0;
   m_punPressedKeys[DIRECTION_BACKWARD] = 0;
   m_punPressedKeys[DIRECTION_LEFT]     = 0;
   m_punPressedKeys[DIRECTION_RIGHT]    = 0;

   m_pcExperimentLoopFunctions = static_cast<CExperimentLoopFunctionsNop *>(
        &CSimulator::GetInstance().GetLoopFunctions());

   m_bDrawRobotLabel = m_pcExperimentLoopFunctions->IsDrawRobotLabel();

   /* Register function to draw entity name */
   RegisterUserFunction<CManualControlQTUserFunctionsNop,CEPuckEntity>(&CManualControlQTUserFunctionsNop::Draw);
   RegisterUserFunction<CManualControlQTUserFunctionsNop,CEPuckChargerEntity>(&CManualControlQTUserFunctionsNop::Draw);
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctionsNop::KeyPressed(QKeyEvent* pc_event) {
   /* Make sure that a controller was set */
   if(!m_pcController) {
      GetQTOpenGLWidget().KeyPressed(pc_event);
      return;
   }
   // switch(pc_event->key()) {
   //    case Qt::Key_I:
   //       /* Forwards */
   //       m_punPressedKeys[DIRECTION_FORWARD] = 1;
   //       SetDirectionFromKeyEvent();
   //       break;
   //    case Qt::Key_K:
   //       /* Backwards */
   //       m_punPressedKeys[DIRECTION_BACKWARD] = 1;
   //       SetDirectionFromKeyEvent();
   //       break;
   //    case Qt::Key_J:
   //       /* Left */
   //       m_punPressedKeys[DIRECTION_LEFT] = 1;
   //       SetDirectionFromKeyEvent();
   //       break;
   //    case Qt::Key_L:
   //       /* Right */
   //       m_punPressedKeys[DIRECTION_RIGHT] = 1;
   //       SetDirectionFromKeyEvent();
   //       break;
   //    default:
   //       /* Unknown key */
   //       GetQTOpenGLWidget().KeyPressed(pc_event);
   //       break;
   // }
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctionsNop::KeyReleased(QKeyEvent* pc_event) {
   /* Make sure that a controller was set */
   if(!m_pcController) {
      GetQTOpenGLWidget().KeyReleased(pc_event);
      return;
   }
   // switch(pc_event->key()) {
   //    case Qt::Key_I:
   //       /* Forwards */
   //       m_punPressedKeys[DIRECTION_FORWARD] = 0;
   //       SetDirectionFromKeyEvent();
   //       break;
   //    case Qt::Key_K:
   //       /* Backwards */
   //       m_punPressedKeys[DIRECTION_BACKWARD] = 0;
   //       SetDirectionFromKeyEvent();
   //       break;
   //    case Qt::Key_J:
   //       /* Left */
   //       m_punPressedKeys[DIRECTION_LEFT] = 0;
   //       SetDirectionFromKeyEvent();
   //       break;
   //    case Qt::Key_L:
   //       /* Right */
   //       m_punPressedKeys[DIRECTION_RIGHT] = 0;
   //       SetDirectionFromKeyEvent();
   //       break;
   //    default:
   //       /* Unknown key */
   //       GetQTOpenGLWidget().KeyReleased(pc_event);
   //       break;
   // }
}

/****************************************/
/****************************************/

// void CManualControlQTUserFunctionsNop::EntitySelected(CEntity& c_entity) {
//    /* Make sure the entity is an e-puck */
//    CEPuckEntity* pcFB = dynamic_cast<CEPuckEntity*>(&c_entity);
//    if(!pcFB) return;
//    /* It's an e-puck worker; extract its controller */
//    m_pcController = dynamic_cast<CWorker*>(&pcFB->GetControllableEntity().GetController());
//    /* Tell that e-puck that it is selected */
//    m_pcController->Select();
//    /* Reset key press information */
//    m_punPressedKeys[DIRECTION_FORWARD]  = 0;
//    m_punPressedKeys[DIRECTION_BACKWARD] = 0;
//    m_punPressedKeys[DIRECTION_LEFT]     = 0;
//    m_punPressedKeys[DIRECTION_RIGHT]    = 0;
// }

// /****************************************/
// /****************************************/

// void CManualControlQTUserFunctionsNop::EntityDeselected(CEntity& c_entity) {
//    /* Make sure that a controller was set (should always be true...) */
//    if(!m_pcController) return;
//    /* Tell the e-puck that it is deselected */
//    m_pcController->Deselect();
//    /* Forget the controller */
//    m_pcController = NULL;
// }

// /****************************************/
// /****************************************/

// void CManualControlQTUserFunctionsNop::SetDirectionFromKeyEvent() {
//    /* Forward/backward direction factor (local robot X axis) */
//    SInt32 FBDirection = 0;
//    /* Left/right direction factor (local robot Y axis) */
//    SInt32 LRDirection = 0;
//    /* Calculate direction factor */
//    if(m_punPressedKeys[DIRECTION_FORWARD])  ++FBDirection;
//    if(m_punPressedKeys[DIRECTION_BACKWARD]) --FBDirection;
//    if(m_punPressedKeys[DIRECTION_LEFT])     ++LRDirection;
//    if(m_punPressedKeys[DIRECTION_RIGHT])    --LRDirection;
//    /* Calculate direction */
//    CVector2 cDir =
//       DIRECTION_VECTOR_FACTOR *
//       (CVector2(FBDirection, 0.0f) +
//        CVector2(0.0f, LRDirection));
//    /* Set direction */
//    m_pcController->SetControlVector(cDir);
// }

/****************************************/
/****************************************/

void CManualControlQTUserFunctionsNop::Draw(CEPuckEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the e-puck
    * See also the description in
    * $ argos3 -q e-puck
    */

   if(m_bDrawRobotLabel) {
      
      try {
         CWorker& cController = dynamic_cast<CWorker&>(c_entity.GetControllableEntity().GetController());

         std::string text = c_entity.GetId().c_str();
         // std::string text = "   ";

         /* Append energy dependent action */
         std::ostringstream energyStream;
         // if(cController.IsSharingEnergy()) {
         //    text += ":Sharing";
         // } else 
         if(cController.IsCharging()) {
            text += ":Charging";
         } else if(cController.IsWorking()) {
            text += ":Working";
         } else if(cController.IsMoving()) {
            text += ":Moving";
         } else {
            text += ":Idle";
         }

         /* Append energy level */
         CBatteryEquippedEntity& cBattery = c_entity.GetBatterySensorEquippedEntity();
         Real robot_energy = cBattery.GetAvailableCharge();
         energyStream << std::fixed << std::setprecision(1) << robot_energy;
         text += "(" + energyStream.str() + ")";
         // text += energyStream.str();
         
         /* Apply text color according to energy level */
         
         CColor cColor = CColor::BLACK;
         // CColor cColor = CColor(0, 191, 0, 255); // GREEN
         // if(robot_energy == 0) {
         //    cColor = CColor(0, 0, 0, 255); // BLACK
         // } else {
         //    // GREEN --- ORANGE --- RED
         //    UInt8 redValue = std::min(1.0, 2.0 - 2 * (robot_energy/cBattery.GetFullCharge())) * 255;
         //    UInt8 greenValue = std::min(1.0, robot_energy/cBattery.GetFullCharge() + 0.5) * 191;
         //    cColor = CColor(redValue, greenValue, 0, 255);
         //    // LOG << redValue << " " << greenValue << " " << robot_energy/cBattery.GetFullCharge() << std::endl;
         // }

         QFont workerFont("Helvetica [Cronyx]", 10);
         // QFont workerFont("Helvetica [Cronyx]", 15, QFont::Bold);
         DrawText(CVector3(0.0, 0.0, 0.2),   // position
                  text,
                  cColor,
                  workerFont); // text
      } catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While casting robot as a worker", ex);
      } catch(const std::bad_cast& e) {
         std::cout << e.what() << " in Draw (e-puck)" << '\n';
      }
   }
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctionsNop::Draw(CEPuckChargerEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the e-puck
    * See also the description in
    * $ argos3 -q e-puck
    */

   if(m_bDrawRobotLabel) {
      
      try {
         CCharger& cController = dynamic_cast<CCharger&>(c_entity.GetControllableEntity().GetController());

         std::string text = c_entity.GetId().c_str();
         // std::string text = "   ";

         /* Draw energy transfer lines */
         std::vector<std::string> vecEnergyTo = cController.GetEnergyTo();
         std::unordered_map<std::string,RobotPosition> robotPos = m_pcExperimentLoopFunctions->GetRobotPos();
         RobotPosition myPos = robotPos[c_entity.GetId()];
         CRadians cZAngle, cYAngle, cXAngle;
         myPos.orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

         for(const auto& id : vecEnergyTo) {
            if(robotPos.find(id) != robotPos.end()) {
               CVector2 pos = robotPos[id].position;
               /* Find vector from myPos to the other robot pos */
               CVector2 vec = pos - myPos.position;
               /* Convert CVector2 to CVector3 */
               CVector3 vec3 = CVector3(vec.GetX(), vec.GetY(), 0.001);
               /* rotate vector according to cZAngle */
               vec3.RotateZ(-cZAngle);
               /* Create Ray between origin to vec3 */
               CRay3 ray = CRay3(CVector3(0, 0, 0.001), vec3);

               /* Reduce length of vec3 by the e-puck body radius */
               CVector2 vec2 = CVector2(vec3.GetX(), vec3.GetY());

               // /* Draw text in the center point along this vector */
               // std::stringstream oss;
               // oss << std::fixed << std::setprecision(1) << (vec.Length() / cController.GetRABRange() * 100) << "%";
               
               // CVector2 textPos = vec2 / 2;
               // if(hop.ID[0] == 'L' || stoi(c_entity.GetId().substr(1)) < stoi(hop.ID.substr(1))) {
               //    DrawText(CVector3(textPos.GetX(), textPos.GetY(), 0.005), oss.str(), CColor::BLACK);
               // }

               vec2 = reduceLength(vec2, 0.035);

               /* Calculate angle of vec2 */
               CRadians angle = vec2.Angle().SignedNormalize();

               CVector2 arrowTip1 = arrowTip(vec2, 0.05, angle + CRadians::PI - CRadians::PI_OVER_SIX);
               CVector2 arrowTip2 = arrowTip(vec2, 0.05, angle + CRadians::PI + CRadians::PI_OVER_SIX);
               CRay3 ray_1 = CRay3(CVector3(vec2.GetX(), vec2.GetY(), 0.001), CVector3(arrowTip1.GetX(), arrowTip1.GetY(), 0.001));
               CRay3 ray_2 = CRay3(CVector3(vec2.GetX(), vec2.GetY(), 0.001), CVector3(arrowTip2.GetX(), arrowTip2.GetY(), 0.001));

               /* Draw arrow */
               CColor arrow_color = CColor::CYAN;

               DrawRay(ray, arrow_color, 4);
               DrawRay(ray_1, arrow_color, 4);
               DrawRay(ray_2, arrow_color, 4);
            }   
         }
         
         /* Append energy dependent action */
         std::ostringstream energyStream;
         if(cController.IsSharingEnergy()) {
            text += ":Sharing";
         } else if(cController.IsCharging()) {
            text += ":Charging";
         } else if(cController.IsMoving()) {
            text += ":Moving";
         } else {
            text += ":Idle";
         }

         /* Append energy level */
         CBatteryEquippedEntity& cBattery = c_entity.GetBatterySensorEquippedEntity();
         Real robot_energy = cBattery.GetAvailableCharge();
         energyStream << std::fixed << std::setprecision(1) << robot_energy;
         text += "(" + energyStream.str() + ")";
         // text += energyStream.str();
         
         /* Apply text color according to energy level */
         CColor cColor = CColor::BLACK;
         // CColor cColor = CColor(0, 191, 0, 255); // GREEN
         // if(robot_energy == 0) {
         //    cColor = CColor(0, 0, 0, 255); // BLACK
         // } else {
         //    // GREEN --- ORANGE --- RED
         //    UInt8 redValue = std::min(1.0, 2.0 - 2 * (robot_energy/cBattery.GetFullCharge())) * 255;
         //    UInt8 greenValue = std::min(1.0, robot_energy/cBattery.GetFullCharge() + 0.5) * 191;
         //    cColor = CColor(redValue, greenValue, 0, 255);
         //    // LOG << redValue << " " << greenValue << " " << robot_energy/cBattery.GetFullCharge() << std::endl;
         // }

         QFont workerFont("Helvetica [Cronyx]", 10);
         // QFont workerFont("Helvetica [Cronyx]", 15, QFont::Bold);
         DrawText(CVector3(0.0, 0.0, 0.2),   // position
                  text,
                  cColor,
                  workerFont); // text
      } catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While casting robot as a charger", ex);
      } catch(const std::bad_cast& e) {
         std::cout << e.what() << " in Draw (e-puck_charger)" << '\n';
      }
   }
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctionsNop::DrawInWorld() {

   /* 
   * Draw Task
   */

   /* Get the task */
   CSpace::TMapPerType* cTasks;
   bool b_tasks_exists = false;
   std::string strTaskType = m_pcExperimentLoopFunctions->GetTaskType();

   try {
      if(strTaskType == "rectangle_task_no_demand")
         cTasks = &CSimulator::GetInstance().GetSpace().GetEntitiesByType("rectangle_task_no_demand");
      else if(strTaskType == "rectangle_task")
         cTasks = &CSimulator::GetInstance().GetSpace().GetEntitiesByType("rectangle_task");
      b_tasks_exists = true;
   } catch(CARGoSException& ex) {
      std::cout << "No task found in argos file (DrawInWorld)" << std::endl;
   }

   if( b_tasks_exists ) {
      for(CSpace::TMapPerType::iterator it = cTasks->begin();
       it != cTasks->end();
       ++it) {

         /* Draw task */
         CVector2 pos;
         Real demand;

         /* Get handle to the task entity */
         if(strTaskType == "rectangle_task_no_demand") {
            CRectangleTaskNoDemandEntity& cTask = *any_cast<CRectangleTaskNoDemandEntity*>(it->second);
            pos = cTask.GetPosition();
            demand = cTask.GetWorkPerformed();
         } else if(strTaskType == "rectangle_task") {
            CRectangleTaskEntity& cTask = *any_cast<CRectangleTaskEntity*>(it->second);
            pos = cTask.GetPosition();
            demand = cTask.GetDemand();
         } else {
            LOGERR << "Unknown task type: " << strTaskType << std::endl;
         }
  
         /* Draw task info */
         std::ostringstream cText;
         cText.str("");
         // cText << ceil(cTask.GetDemand() / 10);
         cText << demand;
         QFont taskFont("Helvetica [Cronyx]", 15, QFont::Bold);
         // DrawText(CVector3(pos.GetX(), pos.GetY()+cTask.GetRadius()/2, 0.01),
         //          cText.str(),
         //          CColor::BLACK,
         //          taskFont);
         DrawText(CVector3(pos.GetX(), pos.GetY() + 0.6, 0.01),
                  cText.str(),
                  CColor::BLACK,
                  taskFont);

         // cText.str("");
         // cText << cTask.GetCurrentRobotNum() << " / " << cTask.GetMinRobotNum();
         // // LOGERR << "draw " << cTask.GetId() << ", " << cTask.GetCurrentRobotNum() << std::endl;
         // QFont numFont("Helvetica [Cronyx]", 20);
         // // DrawText(CVector3(pos.GetX()-0.3, pos.GetY()-0.25, 0.01),
         // DrawText(CVector3(pos.GetX()-0.3, pos.GetY()+0.38, 0.01),
         //          cText.str(),
         //          CColor::BLACK,
         //          numFont);
      }
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CManualControlQTUserFunctionsNop, "manualcontrol_qtuser_functions_nop")
