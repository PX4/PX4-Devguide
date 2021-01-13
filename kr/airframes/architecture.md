# Airframe 개요

PX4 시스템은 모듈화 구조로 되어 있어 모든 로봇 타입을 단일 코드베이스로 대응합니다.

[![](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggTFI7XG4gIEF1dG9waWxvdC0tPkNvbnRyb2xsZXI7XG4gIFNhZmV0eVBpbG90LS0-Q29udHJvbGxlcjtcbiAgQ29udHJvbGxlci0tPk1peGVyO1xuICBNaXhlci0tPkFjdHVhdG9yIiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggTFI7XG4gIEF1dG9waWxvdC0tPkNvbnRyb2xsZXI7XG4gIFNhZmV0eVBpbG90LS0-Q29udHJvbGxlcjtcbiAgQ29udHJvbGxlci0tPk1peGVyO1xuICBNaXhlci0tPkFjdHVhdG9yIiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0)

## 기본 장비

airframe 섹션에 있는 모든 하드웨어 설정은 다음과 같은 기본 장비 설정을 가정 :

  * Taranis Plus 리모트 컨트롤 (혹은 PPM / S.BUS 출력을 내는 장치)
  * 그라운드 컨트롤 스테이션
    * 삼성 Note 4 혹은 최근 안드로이드 타블릿
    * iPad (Wifi 텔레메트리 아답터 요망)
    * 맥북 혹은 Ubuntu Linux 랩톱
  * 내부 컴퓨터 (소프트웨어 개발자용)
    * MacBook Pro 혹은 Air로 OS X 10.10 이상 지원
    * 최신 노트북으로 Ubuntu Linux 지원(14.04 이상)
  * 보안경
  * 멀티콥터용: 위험한 테스트를 위한 tether

PX4는 다양한 장비들과 함께 사용할 수 있지만 새로 입문하는 개발자는 표준 셋업, Taranis RC와 Note 4 타블릿으로 비교적 저렴하게 사용할 수 있는 키트를 구성하는 것이 좋습니다.
