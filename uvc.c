#include "include/uvc.h"

UVC::UVC()
{

}

UVC::~UVC()
{

}

void UVC::uvc_maincontrol()
{
    float pb,rb,k;
    load_imu();
	// ************ 傾斜角へのオフセット適用 ************
	rb=roll;		//一時退避
	pb=pitch;
	k=sqrt(pitch*pitch+roll*roll);	//合成傾斜角
	if( k>0.033 ){
		k=(k-0.033)/k;
		pitch *=k;
		roll  *=k;
	}
	else{
		pitch =0;
		roll  =0;
	}


	// ************ 傾斜角に係数適用 ************
	rollt =0.25*roll;
	if(balance.sup_foot_==1)	rollt = -rollt;		//横方向符号調整
	pitcht=0.25*pitch;

	if(fwct>landF && fwct<=fwctEnd-landB ){

		// ************ UVC主計算 ************
		k	  = atan ((dyi-sw)/autoH );	//片脚の鉛直に対する開脚角
		kl	  = autoH/cos(k);			//前から見た脚投影長
		ks = k+rollt;					//開脚角に横傾き角を加算
		k  = kl*sin(ks);					//中点から横接地点までの左右距離
		dyi	  = k+sw;					//横方向UVC補正距離
		autoH = kl*cos(ks);				//K1までの高さ更新

		// **** UVC（前後） *****
		k 	  = atan( dxi/autoH );		//片脚のX駆動面鉛直から見た現時点の前後開脚角
		kl	  = autoH/cos(k);			//片脚のX駆動面鉛直から見た脚長
		ks	  = k+pitcht;				//振出角に前後傾き角を加算
		k	  = kl*sin(ks);				//前後方向UVC補正距離
		dxi	  = k;						//前後方向UVC補正距離
		autoH = kl*cos(ks);				//K1までの高さ更新

		// ************ UVC積分値リミット設定 ************
		if(dyi<  0)		dyi=   0;
		if(dyi> 45)		dyi=  45;
		if(dxi<-45)		dxi= -45;
		if(dxi> 45)		dxi=  45;

		// ************ 遊脚側を追従させる ************

		// **** 左右方向 *****
		dyis = dyi;					//遊脚Y目標値

		// **** 前後方向 *****
		dxis = -dxi;				//遊脚X目標値

		// ************ 両脚内側並行補正 ************
		if(jikuasi==0){		//両脚を並行以下にしない
			k = -sw+dyi;	//右足の外側開き具合
			ks=  sw+dyis;	//左足の外側開き具合
		}
		else{
			ks= -sw+dyi;	//左足の外側開き具合
			k =  sw+dyis;	//右足の外側開き具合
		}
		if(k+ks<0)dyis-=k+ks;	//遊脚を平衡に補正
	}
	roll =rb;
	pitch=pb;
}

void UVC::uvc_first_postcontrol()
{

}

void UVC::uvc_second_postcontrol()
{

}

void UVC::load_imu()
{
    roll  = balance.roll_imu_filtered_;
	pitch = balance.pitch_imu_filtered_;
}